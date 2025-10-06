// PID HESAPLAMA
void PID(unsigned int cizgiPozisyonu) {
  int hata = 3500 - cizgiPozisyonu;

  float P = Kp * hata;
  float D = Kd * (hata - oncekiHata);
  oncekiHata = hata;

  int PID_cikis = P + D;

  int solMotPwmDegeri = speed - PID_cikis;
  int sagMotPwmDegeri = speed + PID_cikis;

  if (solMotPwmDegeri > 200) solMotPwmDegeri = 200;
  else if (solMotPwmDegeri < 0) solMotPwmDegeri = 0;

  if (sagMotPwmDegeri > 200) sagMotPwmDegeri = 200;
  else if (sagMotPwmDegeri < 0) sagMotPwmDegeri = 0;

  motorA_ileri(solMotPwmDegeri);
  motorB_ileri(sagMotPwmDegeri);
}