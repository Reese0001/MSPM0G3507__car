from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


def compile_and_run(harness: str, sources: tuple[str, ...], defines: str = ""):
    vsdevcmd = (
        Path(os.environ["ProgramFiles"])
        / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
    )
    with tempfile.TemporaryDirectory() as temp_dir:
        executable = Path(temp_dir) / "harness.exe"
        source_args = " ".join(f'"{PROJECT / source}"' for source in sources)
        command = (
            f'call "{vsdevcmd}" -arch=x64 >nul && '
            f'cl /nologo /std:c11 /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /TC {defines} '
            f'/I"{PROJECT}" "{ROOT / "tests" / harness}" {source_args} '
            f'/Fe"{executable}" && "{executable}"'
        )
        return subprocess.run(
            command,
            cwd=temp_dir,
            capture_output=True,
            text=True,
            errors="replace",
            check=False,
            shell=True,
            executable=os.environ["ComSpec"],
        )


class CleanLineRuntime(unittest.TestCase):
    def test_line_follower_is_the_only_control_algorithm(self):
        result = compile_and_run(
            "line_follower_clean_harness.c",
            (
                "modules/line_tracking/line_follower.c",
                "modules/line_tracking/decoder/line_position.c",
            ),
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_line_speed_tuning_is_centralized(self):
        source = (PROJECT / "modules/line_tracking/line_follower.c").read_text(
            encoding="utf-8"
        )
        config = (PROJECT / "modules/line_tracking/line_tracking_config.h").read_text(
            encoding="utf-8"
        )
        for name in (
            "LINE_KP",
            "LINE_KD",
            "LINE_KYAW",
            "LINE_MOTOR_TURN_SIGN",
        ):
            self.assertIn(name, config)

    def test_drive_is_the_only_motor_authority(self):
        result = compile_and_run(
            "drive_clean_harness.c",
            (
                "modules/motor/drive.c",
                "modules/motor/feedback/motor_feedback.c",
                "modules/motor/feedback/differential_controller.c",
            ),
            "/DDRIVE_HOST_TEST",
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_lap_timer_ignores_start_marker_then_stops_on_return(self):
        result = compile_and_run(
            "lap_tracker_harness.c",
            (
                "modules/line_tracking/lap_tracker.c",
                "modules/line_tracking/stop_line_detector.c",
            ),
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_old_runtime_layers_are_removed(self):
        removed = (
            "app/mailbox",
            "app/control",
            "app/line",
            "app/run",
            "app/safety",
            "modules/line_tracking/controller",
            "modules/line_tracking/recovery",
            "modules/line_tracking/prediction",
            "modules/motor/adapter",
            "modules/motor/safety",
            "modules/mpu6050",
        )
        leftovers = []
        for path in removed:
            folder = PROJECT / path
            leftovers.extend(folder.glob("*.[ch]") if folder.exists() else ())
        self.assertEqual([], leftovers)

    def test_no_dual_mode_or_mailbox_remains(self):
        sources = "\n".join(
            path.read_text(encoding="utf-8", errors="ignore")
            for path in PROJECT.rglob("*.[ch]")
            if "Build_LineFollowing" not in path.parts and "Debug" not in path.parts
        )
        for forbidden in (
            "LINE_FOLLOWING_CONTROL_MODE",
            "LINE_CONTROL_MODE_ASSISTED",
            "AppMailbox_",
            "SafetySupervisor_",
            "Motor_Safety_RequestSpeed",
        ):
            self.assertNotIn(forbidden, sources)

    def test_scheduler_is_a_direct_fixed_period_pipeline(self):
        source = (PROJECT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        for required in (
            "FourLineScanner_Sample",
            "FourLineScanner_GetSnapshot",
            "LineFollower_Step",
            "Drive_SetTarget",
            "Drive_Service",
            "Mpu6050_Service",
            "Dashboard_Render",
        ):
            self.assertIn(required, source)
        for forbidden in ("AppMailbox_", "SafetyRuntime_", "RuntimeObserver_"):
            self.assertNotIn(forbidden, source)
        self.assertNotIn("LineScanner_ReadFrame", source)
        self.assertIn("Ssd1306_FlushNextChunk", source)
        self.assertNotIn("Ssd1306_FlushNextDirtyPage", source)
        self.assertIn("FOUR_LINE_SAMPLE_PERIOD_MS", source)
        self.assertRegex(
            source,
            r"if\s*\(due\(now_ms,\s*last_scanner_ms,\s*"
            r"FOUR_LINE_SAMPLE_PERIOD_MS\)\)[\s\S]{0,180}"
            r"FourLineScanner_Sample\(now_ms\)[\s\S]{0,180}"
            r"sample_and_control\(now_ms\)",
        )

    def test_oled_flush_is_split_into_bounded_chunks(self):
        display = (
            PROJECT / "modules/display/ssd1306/ssd1306.c"
        ).read_text(encoding="utf-8")
        self.assertIn("SSD1306_FLUSH_CHUNK (16U)", display)
        self.assertIn("bool Ssd1306_FlushNextChunk(void)", display)
        self.assertNotIn("bool Ssd1306_FlushNextDirtyPage(void)", display)

    def test_reset_zeroes_motor_before_and_after_configuration(self):
        boot = (PROJECT / "app/boot/app_boot.c").read_text(encoding="utf-8")
        self.assertGreaterEqual(boot.count("Drive_Init();"), 2)
        first_zero = boot.index("Drive_Init();")
        configure = boot.index("Set_Motor(5)")
        second_zero = boot.index("Drive_Init();", first_zero + 1)
        self.assertLess(first_zero, configure)
        self.assertLess(configure, second_zero)

    def test_degraded_imu_is_not_fresh_control_input(self):
        tasks = (PROJECT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        self.assertIn("imu->status.health == MODULE_HEALTH_OK", tasks)

    def test_legacy_speed_and_pwm_entry_points_are_removed(self):
        protocol = (
            (PROJECT / "modules/motor/protocol/motor_protocol.c").read_text(
                encoding="utf-8"
            )
            + (PROJECT / "modules/motor/protocol/motor_protocol.h").read_text(
                encoding="utf-8"
            )
        )
        self.assertNotIn("Contrl_Speed", protocol)
        self.assertNotIn("Contrl_Pwm", protocol)

    def test_dashboard_exposes_request_output_and_imu_use(self):
        source = (PROJECT / "modules/display/dashboard.c").read_text(
            encoding="utf-8"
        )
        for label in ("RUN ", "STOP ", "LINE B", "SPD", "DIF", "DST", "ANG", "YAW", "TURN", "ERR", "TRE"):
            self.assertIn(label, source)
        self.assertIn("Ssd1306_ClearBuffer", source)

    def test_dashboard_exposes_four_channel_preflight_and_frozen_states(self):
        source = (PROJECT / "modules/display/dashboard.c").read_text(
            encoding="utf-8"
        )
        for label in ("X1", "X2", "X3", "X4", "WAIT K1 SAFE", "RUN ", "STOP "):
            self.assertIn(label, source)
        self.assertRegex(
            source,
            r'"X1 %d X2 %d",\s*\(data->raw_x_bits & 0x02U\)[\s\S]*?'
            r'\(data->raw_x_bits & 0x01U\)',
        )
        self.assertRegex(
            source,
            r'"X3 %d X4 %d",\s*\(data->raw_x_bits & 0x04U\)[\s\S]*?'
            r'\(data->raw_x_bits & 0x08U\)',
        )

    def test_obsolete_mux_has_no_runtime_or_build_reference(self):
        scanner = PROJECT / "modules/line_tracking/scanner"
        self.assertFalse((scanner / "line_mux.c").exists())
        self.assertFalse((scanner / "line_mux.h").exists())
        makefile = (PROJECT / "Makefile").read_text(encoding="utf-8")
        self.assertNotIn("line_mux", makefile)
        sources = "\n".join(
            path.read_text(encoding="utf-8", errors="ignore")
            for path in PROJECT.rglob("*.[ch]")
        )
        self.assertNotIn("BSP_LineMux", sources)


if __name__ == "__main__":
    unittest.main()
