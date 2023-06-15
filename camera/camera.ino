
extern "C"
{

void cmain(char *obuf);
  
}

void setup() {
  Serial.begin(9600);
  
  char tmp[256]; 
  cmain(tmp);
  Serial.println(tmp);
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
