// MULTIPEXERDEN VERİLERİ ALMA
void sensorleriOku() {
  for (int i = 0; i < 8; i++) {
    digitalWrite(S0, i & 0x01);
    digitalWrite(S1, (i >> 1) & 0x01);
    digitalWrite(S2, (i >> 2) & 0x01);
    delayMicroseconds(10);
    sensorDegerleri[i] = analogRead(Z_PIN);
  }
}

// PID ICIN POZISYON HESAPLAMA
unsigned int readLineBlack(uint16_t *sensorDegerleri) {
  long toplam = 0;
  long toplamDeger = 0;

  int solUc = sensorDegerleri[0];
  int sagUc = sensorDegerleri[7];

  for (int i = 0; i < 8; i++) {
    int deger = sensorDegerleri[i];
    toplam += (long)deger * i * 1000;
    toplamDeger += deger;
  }

  if (toplamDeger == 0) {
    return (sonPozisyon < 3500) ? 0 : 7000;
  }

  if (toplamDeger < 2500) {
    if (sagUc > 1000 && solUc < 300) return 7000;
    if (solUc > 1000 && sagUc < 300) return 0;
    return sonPozisyon;
  }

  sonPozisyon = toplam / toplamDeger;
  return sonPozisyon;
}