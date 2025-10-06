// KORDINAT GUNCELLEME
void collision() {
  unsigned long simdikiZaman = millis();

  if (simdikiZaman - sonCollisionZamani < 2000) {
    return;
  }

  int8_t siyahSensorSayisi = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorDegerleri[i] > 2200) {
      siyahSensorSayisi++;
    }
  }

  // KESISIM VARSA
  if (siyahSensorSayisi >= 6) {
    if (yon == KUZEY) y++;
    else if (yon == GUNEY) y--;
    else if (yon == DOGU) x++;
    else if (yon == BATI) x--;

    // ID:cihazID>>X=<x>,Y=<y>,D=<yon>
    String message = String("ID:") + deviceID + ">>" + "X=" + x + ",Y=" + y + ",D=" + yonToString(yon);
    webSocket.sendTXT(message);

    motorA_ileri(speed);
    motorB_ileri(speed);
    delay(700);  // Daha rahat dönüş için çizgiyi geçsin.

    if (x == target_x && y == target_y) {
      hedefeUlasildi = true;
      motorAll_dur();
      return;
    }

    hareketPlanla();
    sonCollisionZamani = simdikiZaman;
  }
}

// HAREKET PLANMA
void hareketPlanla() {
  if (y != target_y) {
    if (y < target_y) hedefeGit(KUZEY);
    else hedefeGit(GUNEY);
  } else if (x != target_x) {
    if (x < target_x) hedefeGit(DOGU);
    else hedefeGit(BATI);
  }
}

// HEDEFE GITME
void hedefeGit(Yon hedefYon) {
  if (yon == hedefYon) return;

  if ((yon == KUZEY && hedefYon == DOGU) || (yon == DOGU && hedefYon == GUNEY) || (yon == GUNEY && hedefYon == BATI) || (yon == BATI && hedefYon == KUZEY)) {
    sagaDon();
  } else if ((yon == KUZEY && hedefYon == BATI) || (yon == BATI && hedefYon == GUNEY) || (yon == GUNEY && hedefYon == DOGU) || (yon == DOGU && hedefYon == KUZEY)) {
    solaDon();
  } else {
    geriDon();
  }
  yon = hedefYon;
}

String yonToString(Yon yon) {
  switch (yon) {
    case KUZEY: return "KUZEY";
    case GUNEY: return "GUNEY";
    case DOGU: return "DOGU";
    case BATI: return "BATI";
    default: return "BILINMEYEN";
  }
}