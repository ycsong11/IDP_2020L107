class chassis {
  public:

}

class rescue_arm {
  public:

}

class sensors {
  public:

}


void state0(int* state)
{
  
}

void setup() {
  // put your setup code here, to run once:

}

int state = 0;
void loop() {

  switch(state){
    case 0:
      state0(&state); //can't make a 'state' class because .do() will all be different
      break;
    case 1:
      state1()
      break
      
  }
}
