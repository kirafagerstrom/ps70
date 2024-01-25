#include "Keyboard.h"


// declare global vars
const int safetyPin = 12;
const int profile = A2;

// define a class for our macro buttons
class MacroButton{

  // declare private variables for this class
  private:
  int id;
  int pin; // the control pin to see if this button has been pressed
  int state;
  int cooldown;

  // constructor
  public:
  MacroButton(int id_number, int read_pin){
    id = id_number;
    pin = read_pin;
    pinMode(pin, INPUT_PULLUP);
    state = digitalRead(pin);
    cooldown = 0;
  }

  // bool isPressed(): checks if this button has been pressed.
  // if pressed, resets state and cooldown variables to prevent double-presses
  //     and returns `true` 
  // else returns `false`
  bool isPressed(){
    int new_state = digitalRead(pin);
    if ((state != new_state) && (cooldown + 100 < millis())) {
      Serial.print("M");
      Serial.print(id);
      Serial.println(" pressed");
      state = new_state;
      cooldown = millis();
      return true;
    } else {
      return false;
    }
  };
};

// initialize our macro button on pin 3
MacroButton M1(1, 2);
MacroButton M2(2, A5);
MacroButton M3(3, A4);
MacroButton M4(4, A3);
MacroButton M5(5, 7);
MacroButton M6(6, 9);
MacroButton M7(7, 10);
MacroButton M8(8, 11);


void setup() {

  pinMode(safetyPin, INPUT_PULLUP);
  pinMode(profile, INPUT_PULLUP);

  Keyboard.begin();
  Serial.begin(115200);
  delay(100);
}


void loop() {

  if (digitalRead(safetyPin) < 1) {
    Serial.println("SAFETY");
    Keyboard.end();
    delay(100);
    Serial.println("Keyboard interrupted. Please reset to continue.");
    delay(10000);
  }

  if (digitalRead(profile) < 1) {
    if (M1.isPressed()) {
      macro1();
    }
    if (M2.isPressed()) {
      macro2();
    }
    if (M3.isPressed()) {
      macro3();
    }
    if (M4.isPressed()) {
      macro4();
    }
    if (M5.isPressed()) {
      macro5();
    }
    if (M6.isPressed()) {
      macro6();
    }
    if (M7.isPressed()) {
      macro7();
    }
    if (M8.isPressed()) {
      macro8();
    }
  } else {
    if (M1.isPressed()) {
      macro9();
    }
    if (M2.isPressed()) {
      macro10();
    }
    if (M3.isPressed()) {
      macro11();
    }
    if (M4.isPressed()) {
      macro12();
    }
    if (M5.isPressed()) {
      macro13();
    }
    if (M6.isPressed()) {
      macro14();
    }
    if (M7.isPressed()) {
      macro15();
    }
    if (M8.isPressed()) {
      macro16();
    }
  }
}

void openApp(char* name) {
  Keyboard.press(131);
  Keyboard.press(32);
  Keyboard.releaseAll();
  delay(50);
  Keyboard.println(name);
  Keyboard.releaseAll();
}

// MACRO FUNCTIONS

void macro1() {
  Keyboard.println("hello world");
  Keyboard.releaseAll();
}

void macro2() {
  Keyboard.press(131);
  Keyboard.press('z');
  Keyboard.releaseAll();
}

void macro3() {
  Keyboard.press(129);
  Keyboard.press(131);
  Keyboard.press('z');
  Keyboard.releaseAll();
}

void macro4() {
  Keyboard.press(131);
  Keyboard.press('s');
  Keyboard.releaseAll();
}

void macro5() {
  openApp("Spotify");
}

void macro6() {
  openApp("Safari");
  delay(1000);
  Keyboard.press(131);
  Keyboard.press('n');
  Keyboard.releaseAll();
  delay(50);
  Keyboard.print("https://nathanmelenbrink.github.io/ps70/index.html");
  Keyboard.write(176);
  Keyboard.releaseAll();
}

void macro7() {
  openApp("Spotify");
  Keyboard.write(176);
  Keyboard.releaseAll();
}

void macro8() {
  openApp("Safari");
  delay(1000);
  Keyboard.press(131);
  Keyboard.press('n');
  Keyboard.releaseAll();
  delay(50);
  Keyboard.print("https://c2bstewart.github.io/PS70/index.html");
  Keyboard.write(176);
  Keyboard.releaseAll();
}

void macro9() {
  Keyboard.write('e');
  Keyboard.press(130);
  Keyboard.press('p');
  Keyboard.releaseAll();
  Keyboard.write(49);
  Keyboard.write(176);
  Keyboard.releaseAll();
}

void macro10() {
  Keyboard.press(129);
  Keyboard.press('a');
  Keyboard.releaseAll();
  Keyboard.press(130);
  Keyboard.press('e');
  Keyboard.releaseAll();
  Keyboard.write(57);
  Keyboard.write(48);
  Keyboard.write(49);
  Keyboard.write(176);
  Keyboard.releaseAll();
}

void macro11() {
  Keyboard.press(129);
  Keyboard.press('a');
  Keyboard.releaseAll();
  Keyboard.press(130);
  Keyboard.press('e');
  Keyboard.releaseAll();
  Keyboard.write(57);
  Keyboard.write(48);
  Keyboard.write(50);
  Keyboard.write(176);
  Keyboard.releaseAll(); 
}

void macro12() {
  // Keyboard.press(129);
  // Keyboard.press('a');
  // Keyboard.releaseAll();
  // Keyboard.press(130);
  // Keyboard.press('e');
  // Keyboard.write("903");
  // Keyboard.write(176);
  // Keyboard.releaseAll(); 
}

void macro13() {
  Keyboard.press(129);
  Keyboard.press('a');
  Keyboard.releaseAll();
  Keyboard.press(130);
  Keyboard.press(50);
  Keyboard.releaseAll();
  Keyboard.write(176);
  Keyboard.write('r');
  Keyboard.write('q');
  Keyboard.write(214);
  Keyboard.write('l');
  Keyboard.releaseAll();
}

void macro14() {
  Keyboard.press(128);
  Keyboard.press('g');
  Keyboard.releaseAll();
  Keyboard.write('o');
  Keyboard.write('i');
  Keyboard.write(48);
  Keyboard.write(176);
  Keyboard.releaseAll();
}

void macro15() {
  Keyboard.println("macro15");
  Keyboard.releaseAll();
}

void macro16() {
  openApp("Safari");
  delay(1000);
  Keyboard.press(131);
  Keyboard.press('n');
  Keyboard.releaseAll();
  delay(50);
  Keyboard.print("https://www.youtube.com/watch?v=dQw4w9WgXcQ");
  Keyboard.write(176);
  Keyboard.releaseAll();
}