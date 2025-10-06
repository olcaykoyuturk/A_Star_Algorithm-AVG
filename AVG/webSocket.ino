void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      Serial.println("WebSockete baglandi.");
      
      // SEND ID TO SERVER
      String idMessage = "&ID:" + String(deviceID);
      webSocket.sendTXT(idMessage);
      break;
    }
      
    case WStype_DISCONNECTED:
      Serial.println("WebSocket bağlantisi koptu.");
      break;
      
    case WStype_TEXT: {
      String message = (char*)payload;
      Serial.printf("Server: %s\n", message.c_str());
      
      // IDENTITY VERIFICATION CHECK
      if (message.indexOf("Kimlik kaydedildi") != -1) {
        Serial.println("Server >> ID'yi tanidi.");
      }
      
      // AVG TARGET LOCATION
      if (message.startsWith("TARGET:")) {
        int commaIndex = message.indexOf(',', 7);
        if (commaIndex != -1) {
          target_x = message.substring(7, commaIndex).toInt();
          target_y = message.substring(commaIndex + 1).toInt();
          Serial.printf("Hedef güncellendi: X=%d, Y=%d\n", target_x, target_y);
          
          if (target_x == x && target_y == y) {
            hedefeUlasildi = true;
          } else {
            hedefeUlasildi = false;
            hareketPlanla();
            break;
          }
        }
      }
      
      // ELEKTROMAGNET COMMAND
      else if (message.startsWith("MAGNET:")) {
        String status = message.substring(7);
        if (status == "ON") {
          digitalWrite(MOSFET_PIN, HIGH);
        }
        else if (status == "OFF") {
          digitalWrite(MOSFET_PIN, LOW);
        }
      }
      break;
    }
    
    default:
      break;
  }
}

// CONNECT WIFI
void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("WiFi bağlanıyor...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi baglandi: %s\n", WiFi.localIP().toString().c_str());
}

// CONNECT WEB SOCKET
void setupWebSocket() {
  webSocket.begin("192.168.4.1", 80, "/ws");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}