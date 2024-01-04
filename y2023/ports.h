#ifndef Y2023_PORTS_H_
#define Y2023_PORTS_H_

struct ports {
  struct driver {
    static constexpr int kXbox_DSPort = 0;
  };

  struct drivetrain {
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
    static constexpr int kPivot_CANID = 15;
  };

  struct telescope {
    static constexpr int kTelescoping_CANID = 16;
  };

  struct conveyor {
    static constexpr int kLeftRoller_PWM = 8;
    static constexpr int kRightRoller_PWM = 9;
    static constexpr int kBelt_PWM = 7;

    static constexpr int kProximity_DIO = 9;
  };

  struct gripper {
    static constexpr int kCANID = 20;
  };

  struct wrist {
    static constexpr int kCANID = 21;
  };

  struct intake {
    static constexpr int kDeploy_CANID = 14;
    static constexpr int kRoller_CANID = 17;
  };

  struct leds {
    static constexpr int kPWMPort = 6;
  };
};

#endif  // Y2023_PORTS_H_