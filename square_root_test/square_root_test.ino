void setup() {
  // put your setup code here, to run once:
  



}

extern "C" int testroot(void);

extern "C" int xcmain(char *obuf);

extern "C" 
unsigned int * test_transform();

extern "C"             void test_ook();
void loop() {
  // put your main code here, to run repeatedly:
  Serial.begin(9600);
  Serial.println("start");

  char tmp[256];
  xcmain(tmp);
  Serial.println(tmp);
  

  
  long long s=millis();
  int i;
  for (i=0;i<32000;i++)test_transform();
  long long e=millis();
  int ee=e;
  Serial.println(ee);
  
  Serial.println(testroot());
  for(;;);
}
