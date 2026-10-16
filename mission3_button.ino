/*
  Four-button controller for the separate ATmega328-AU button board.

  Button wiring on this board:
    KEY1 -> A0 / PC0, switch to GND
    KEY2 -> A1 / PC1, switch to GND
    KEY3 -> A2 / PC2, switch to GND
    KEY4 -> A3 / PC3, switch to GND

  The internal pull-ups are enabled, so no external pull-up resistors are
  required. UART commands match the receiver in mission3.ino on the Uno.
*/

const uint8_t KEY_PINS[4] = { A0, A1, A2, A3 };
const unsigned long DEBOUNCE_MS = 25;

struct ButtonState {
  uint8_t raw;
  uint8_t stable;
  unsigned long changedAt;
};

ButtonState buttons[4] = {
  { HIGH, HIGH, 0 },
  { HIGH, HIGH, 0 },
  { HIGH, HIGH, 0 },
  { HIGH, HIGH, 0 }
};

uint8_t nextTask = 0;  // 0=A, 1=B, 2=C

void sendCommand(char command)
{
  Serial.write(command);
  Serial.write('\n');
}

void onButtonPressed(uint8_t index)
{
  switch (index) {
    case 0:  // KEY1: cycle A -> B -> C -> A
      sendCommand('A' + nextTask);
      nextTask = (nextTask + 1) % 3;
      break;

    case 1:  // KEY2: start/stop recording
      sendCommand('R');
      break;

    case 2:  // KEY3: play the recorded motion
      sendCommand('P');
      break;

    case 3:  // KEY4: return to home position
      sendCommand('M');
      break;
  }
}

void scanButtons()
{
  const unsigned long nowMs = millis();

  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t reading = digitalRead(KEY_PINS[i]);

    if (reading != buttons[i].raw) {
      buttons[i].raw = reading;
      buttons[i].changedAt = nowMs;
    }

    if (reading != buttons[i].stable &&
        nowMs - buttons[i].changedAt >= DEBOUNCE_MS) {
      buttons[i].stable = reading;

      // INPUT_PULLUP means a press is a HIGH-to-LOW transition.
      if (buttons[i].stable == LOW) {
        onButtonPressed(i);
      }
    }
  }
}

void setup()
{
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(KEY_PINS[i], INPUT_PULLUP);
  }

  // Same baud rate as mission3.ino. Send nothing at startup; only key events.
  Serial.begin(9600);
}

void loop()
{
  scanButtons();
}
