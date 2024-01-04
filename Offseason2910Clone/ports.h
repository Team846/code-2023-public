#ifndef Offseason2910Clone_PORTS_H_
#define Offseason2910Clone_PORTS_H_

struct ports {
  struct driver {
    static constexpr int kXbox_DSPort = 0;
  };

  struct drivetrain {  // TODO: change all ports (preferably, when setting them,
                       // make them ascending in this order)
    static constexpr int kFLDrive_CANID = 12;
    static constexpr int kFRDrive_CANID = 9;
    static constexpr int kBLDrive_CANID = 6;
    static constexpr int kBRDrive_CANID = 3;

    static constexpr int kFLSteer_CANID = 11;
    static constexpr int kFRSteer_CANID = 8;
    static constexpr int kBLSteer_CANID = 5;
    static constexpr int kBRSteer_CANID = 2;

    static constexpr int kFLCANCoder_CANID = 13;
    static constexpr int kFRCANCoder_CANID = 10;
    static constexpr int kBLCANCoder_CANID = 7;
    static constexpr int kBRCANCoder_CANID = 4;
  };

  struct operator_ {
    static constexpr int kXbox_DSPort = 1;
  };

  struct pivot {
    static constexpr int kL1_CANID = 25;  // TODO: change port
    static constexpr int kL2_CANID = 26;  // TODO: change port
    static constexpr int kR1_CANID = 27;  // TODO: change port
    static constexpr int kR2_CANID = 28;  // TODO: change port
  };

  struct telescope {
    static constexpr int kTele1_CANID = 31;  // TODO: change port
    static constexpr int kTele2_CANID = 32;  // TODO: change port
  };

  struct wrist {
    static constexpr int kLeft_CANID = 17;  // TODO change port
  };

  struct roller {
    static constexpr int kCANID = 20;  // TODO: change port
  };

  struct leds {
    static constexpr int kPWMPort = 6;  // TODO: change port
  };
};

#endif  // Offseason2910Clone_PORTS_H_