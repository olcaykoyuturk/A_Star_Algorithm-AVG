#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include <queue>
#include <algorithm>

const int GRID_W = 5;
const int GRID_H = 5;
const int LOOP_MS = 500;

const char* ap_ssid = "AGV_Network";
const char* ap_pass = "12345678";

AsyncWebServer server(80);
AsyncWebSocket wsUI("/wsUI");
AsyncWebSocket wsAGV("/wsAGV");

typedef struct { int x, y; } Point;

std::map<int, Point> agvStatus;
std::map<int, Point> agvTargets;
std::map<int, Point> prevPos;
std::map<int, AsyncWebSocketClient*> agvClients;
std::map<int, bool> awaitingAck;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>AGV Dashboard & Map</title>
</head>
<body>
  <h2>AGV Dashboard</h2>
  <div id="panels"></div>

  <h2>AGV Grid (5×5)</h2>
  <pre id="grid" style="font-family: monospace; font-size:16px; line-height:1.2;"></pre>

<script>
  const W = 5, H = 5;
  let blockedCells = [];
  let positions = {};
  let targets   = {};

  function initGrid() {
    const grid = Array.from({ length: H }, () => Array(W).fill('.'));
    blockedCells.forEach(b => { if (b.x>=0 && b.x<W && b.y>=0 && b.y<H) grid[b.y][b.x] = '#'; });
    Object.values(targets).forEach(t => { if (t.x>=0&&t.x<W&&t.y>=0&&t.y<H) grid[t.y][t.x] = '*'; });
    Object.entries(positions).forEach(([id,p]) => { if (p.x>=0&&p.x<W&&p.y>=0&&p.y<H) grid[p.y][p.x] = id; });
    return grid;
  }

  function renderGrid() {
    const grid = initGrid();
    let text = '';
    for (let y=0; y<H; y++) {
      for (let x=0; x<W; x++) text += grid[y][x] + ' ';
      text += '\n';
    }
    document.getElementById('grid').innerText = text;
  }

  function updatePanel(data) {
    let panel = document.getElementById('agv'+data.id);
    if (!panel) {
      panel = document.createElement('div');
      panel.id = 'agv'+data.id;
      panel.innerHTML = '<h3>AGV '+data.id+'</h3>' +
        'Pos: (<span class="posx"></span>,<span class="posy"></span>)<br>' +
        'Target: X:<input class="tx" size=1> Y:<input class="ty" size=1> ' +
        '<button onclick="send('+data.id+')">Set</button>';
      document.getElementById('panels').appendChild(panel);
    }
    panel.querySelector('.posx').innerText = data.x;
    panel.querySelector('.posy').innerText = data.y;
  }

  function send(id) {
    const panel = document.getElementById('agv'+id);
    const x = parseInt(panel.querySelector('.tx').value,10);
    const y = parseInt(panel.querySelector('.ty').value,10);
    ws.send(JSON.stringify({ id:id, target:{ x:x, y:y } }));
  }

  const ws = new WebSocket('ws://'+location.host+'/wsUI');
  ws.onmessage = ev => {
    const data = JSON.parse(ev.data);
    if (Array.isArray(data.blocked)) {
      blockedCells = [];
      data.blocked.forEach(b=>blockedCells.push({x:b.x,y:b.y}));
    }
    if (data.x!==undefined && data.y!==undefined) {
      positions[data.id] = { x:data.x, y:data.y };
      updatePanel(data);
    }
    if (data.target) targets[data.id] = { x:data.target.x, y:data.target.y };
    renderGrid();
  };
  renderGrid();
</script>
</body>
</html>
)rawliteral";

bool valid(int x,int y){ return x>=0 && x<GRID_W && y>=0 && y<GRID_H; }

// A* node
struct Node { int x, y, g, f; };

// A* pathfinding
std::vector<Point> AStar(Point start, Point goal, bool blocked[GRID_W][GRID_H]) {
  bool closed[GRID_W][GRID_H] = {};
  int gScore[GRID_W][GRID_H];
  Point parent[GRID_W][GRID_H];
  for (int i=0; i<GRID_W; i++) for (int j=0; j<GRID_H; j++) gScore[i][j] = INT_MAX;

  struct FCompare { bool operator()(const Node &a, const Node &b) const { return a.f > b.f; } } cmp;
  std::priority_queue<Node,std::vector<Node>,FCompare> open(cmp);

  gScore[start.x][start.y] = 0;
  open.push({ start.x, start.y, 0, abs(start.x-goal.x)+abs(start.y-goal.y) });
  const int dx[4]={1,-1,0,0}, dy[4]={0,0,1,-1};

  while (!open.empty()) {
    Node cur = open.top(); open.pop();
    if (cur.x==goal.x && cur.y==goal.y) break;
    if (closed[cur.x][cur.y]) continue;
    closed[cur.x][cur.y] = true;
    for (int k=0; k<4; k++) {
      int nx=cur.x+dx[k], ny=cur.y+dy[k];
      if (!valid(nx,ny) || blocked[nx][ny] || closed[nx][ny]) continue;
      int ng = cur.g + 1;
      if (ng < gScore[nx][ny]) {
        gScore[nx][ny] = ng;
        parent[nx][ny] = { cur.x, cur.y };
        open.push({ nx, ny, ng, ng + abs(nx-goal.x)+abs(ny-goal.y) });
      }
    }
  }

  std::vector<Point> path;
  if (gScore[goal.x][goal.y] == INT_MAX) return path;
  for (Point p=goal; !(p.x==start.x && p.y==start.y); p=parent[p.x][p.y]) path.push_back(p);
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  return path;
}

void onUiEvent(AsyncWebSocket* s, AsyncWebSocketClient* client, AwsEventType t, void*, uint8_t* data, size_t len) {
  if (t != WS_EVT_DATA) return;
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, data, len)) return;
  int id = doc["id"];
  agvTargets[id] = { doc["target"]["x"], doc["target"]["y"] };
  Serial.printf("[UI] Set AGV#%d target to (%d,%d)\n", id, agvTargets[id].x, agvTargets[id].y);
  s->textAll((char*)data, len);
}

void onAgvEvent(AsyncWebSocket*, AsyncWebSocketClient* cli, AwsEventType t, void*, uint8_t* data, size_t len) {
  if (t != WS_EVT_DATA) return;
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, data, len)) return;
  int id = doc["id"];
  Point newPos = { doc["x"], doc["y"] };

  if (awaitingAck[id]) awaitingAck[id] = false;

  agvStatus[id]   = newPos;
  agvClients[id]  = cli;
  wsUI.textAll((char*)data, len);
}

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ap_ssid, ap_pass);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
    req->send_P(200, "text/html", index_html);
  });
  wsUI.onEvent(onUiEvent);
  wsAGV.onEvent(onAgvEvent);
  server.addHandler(&wsUI);
  server.addHandler(&wsAGV);
  server.begin();
  Serial.println("Server ready");
}

void loop() {
  static unsigned long lastLoop = 0;
  unsigned long now = millis();
  if (now - lastLoop < LOOP_MS) return;
  lastLoop = now;

  bool blocked[GRID_W][GRID_H] = {};
  // dynamic blocks
  for (auto &s : agvStatus) blocked[s.second.x][s.second.y] = true;
  blocked[1][1] = blocked[1][3] = blocked[3][3] = blocked[3][1] = true;

  // broadcast blocked map
  StaticJsonDocument<200> bdoc;
  JsonArray barr = bdoc.createNestedArray("blocked");
  for (int x=0; x<GRID_W; x++) {
    for (int y=0; y<GRID_H; y++) {
      if (blocked[x][y]) {
        JsonObject o = barr.createNestedObject();
        o["x"] = x;
        o["y"] = y;
      }
    }
  }
  char bbuf[256]; size_t blen = serializeJson(bdoc, bbuf);
  wsUI.textAll(bbuf, blen);

  // path replanning and move commands
  std::vector<int> ids;
  for (auto &e : agvTargets) ids.push_back(e.first);
  std::sort(ids.begin(), ids.end());

  for (int id : ids) {
    if (!agvStatus.count(id)) continue;
    if (awaitingAck[id]) continue;

    Point start = agvStatus[id];
    Point goal  = agvTargets[id];
    Point prev  = prevPos.count(id) ? prevPos[id] : start;

    // adjust blocked for planning
    blocked[start.x][start.y] = false;
    if (valid(prev.x,prev.y) && (prev.x!=start.x||prev.y!=start.y))
      blocked[prev.x][prev.y] = true;
    bool wasGoal = blocked[goal.x][goal.y];
    blocked[goal.x][goal.y] = false;

    auto path = AStar(start, goal, blocked);

    blocked[goal.x][goal.y] = wasGoal;
    blocked[start.x][start.y] = true;

    if (path.size() > 1) {
      Point nxt = path[1];
      if (!(nxt.x==prev.x && nxt.y==prev.y) && !blocked[nxt.x][nxt.y]) {
        StaticJsonDocument<64> out;
        out["move"]["x"] = nxt.x;
        out["move"]["y"] = nxt.y;
        char buf[64]; size_t n = serializeJson(out, buf);
        agvClients[id]->text(buf, n);
        wsUI.textAll(buf, n);
        Serial.printf("[Move] AGV#%d -> (%d,%d)\n", id, nxt.x, nxt.y);

        awaitingAck[id] = true;
        prevPos[id]    = start;
      }
    }
  }
}
