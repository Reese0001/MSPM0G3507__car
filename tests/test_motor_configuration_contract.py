from pathlib import Path
import re
import unittest


PROJECT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


def read(relative: str) -> str:
    return (PROJECT / relative).read_text(encoding="utf-8", errors="ignore")


class MotorConfigurationContract(unittest.TestCase):
    def setUp(self):
        self.protocol_h = read("modules/motor/protocol/motor_protocol.h")
        self.protocol_c = read("modules/motor/protocol/motor_protocol.c")
        self.configuration_h = read("modules/motor/configuration/motor_configuration.h")
        self.configuration_c = read("modules/motor/configuration/motor_configuration.c")
        self.boot_h = read("app/boot/app_boot.h")
        self.boot_c = read("app/boot/app_boot.c")

    def function_body(self, name: str) -> str:
        match = re.search(r"\b" + name + r"\s*\([^)]*\)\s*\{", self.protocol_c)
        self.assertIsNotNone(match, name)
        start = match.end()
        depth = 1
        for index in range(start, len(self.protocol_c)):
            if self.protocol_c[index] == "{":
                depth += 1
            elif self.protocol_c[index] == "}":
                depth -= 1
                if depth == 0:
                    return self.protocol_c[start:index]
        self.fail(f"unterminated {name}")

    def test_configuration_protocol_senders_are_bounded_bool_functions(self):
        names = (
            "send_motor_type",
            "send_pulse_phase",
            "send_pulse_line",
            "send_wheel_diameter",
            "send_motor_deadzone",
        )
        for name in names:
            self.assertRegex(self.protocol_h, rf"\bbool\s+{name}\s*\(")
            body = self.function_body(name)
            self.assertIn("Motor_Usart_SendArrayBounded", body)
            self.assertNotIn("Send_Motor_ArrayU8", body)
            self.assertRegex(body, r"length\s*<=\s*0")
            self.assertRegex(body, r"length\s*>=\s*\(int\)sizeof\(send_buff\)")

    def test_set_motor_returns_bool_and_stops_on_failed_frame(self):
        self.assertRegex(self.configuration_h, r"\bbool\s+Set_Motor\s*\(")
        self.assertRegex(self.configuration_c, r"\bbool\s+Set_Motor\s*\(")
        self.assertIn("if (!ok) return false;", self.configuration_c)
        self.assertIn("return false;", self.configuration_c)
        self.assertIn("return true;", self.configuration_c)
        l520 = self.configuration_c.split("else if(MOTOR_TYPE == 5)", 1)[1]
        self.assertEqual(l520.count("delay_ms(100)"), 5)

    def test_boot_records_configuration_and_exposes_read_only_status(self):
        self.assertRegex(self.boot_h, r"\bbool\s+AppBoot_IsMotorConfigured\s*\(void\)")
        self.assertRegex(self.boot_h, r"\bbool\s+AppBoot_IsDisplayReady\s*\(void\)")
        for event in ("BOOT", "OLED OK", "OLED FAIL", "AUTO START", "MOTOR CFG"):
            self.assertIn(event, self.boot_c)
        self.assertTrue("CFG OK" in self.boot_c or "UART TIMEOUT" in self.boot_c)
        self.assertIn("Set_Motor(5)", self.boot_c)
        self.assertIn("RuntimeLog_Draw", self.boot_c)
        self.assertIn("Ssd1306_FlushDirty", self.boot_c)

    def test_boot_does_not_arm_or_request_motor_speed(self):
        init_body = self.boot_c.split("void AppBoot_Init(void)", 1)[1]
        self.assertNotIn("Motor_Safety_Arm", init_body)
        self.assertNotIn("Motor_Safety_Request", init_body)


if __name__ == "__main__":
    unittest.main()
