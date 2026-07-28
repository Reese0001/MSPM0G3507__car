from pathlib import Path
import unittest


PROJECT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class BootTimebaseContract(unittest.TestCase):
    def test_microsecond_counter_starts_before_motor_configuration(self):
        timer = (PROJECT / "modules/time/timer.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        app = (PROJECT / "app/boot/app_boot.c").read_text(
            encoding="utf-8", errors="ignore"
        )

        self.assertIn(
            "DL_TimerG_startCounter(MICROSECOND_TIMEBASE_INST)", timer
        )
        self.assertLess(app.index("Timer_Init()"), app.index("Set_Motor(5)"))


if __name__ == "__main__":
    unittest.main()
