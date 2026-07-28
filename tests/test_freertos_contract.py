from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"
REPOSITORY = ROOT.parent


class FreeRtosContract(unittest.TestCase):
    def test_static_only_1khz_kernel_and_safe_start(self):
        config = (ROOT / "FreeRTOSConfig.h").read_text(encoding="utf-8")
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        safety_runtime = (
            ROOT / "app/safety/safety_runtime.c"
        ).read_text(encoding="utf-8")
        main = (ROOT / "empty.c").read_text(encoding="utf-8")

        self.assertIn("#define configTICK_RATE_HZ 1000", config)
        self.assertIn("#define configSUPPORT_STATIC_ALLOCATION 1", config)
        self.assertIn("#define configSUPPORT_DYNAMIC_ALLOCATION 0", config)
        self.assertIn("xTaskCreateStatic", tasks)
        self.assertIn(
            "BootTrace_Fatal(BOOT_FAULT_STACK_OVERFLOW)",
            tasks,
        )
        self.assertIn("vTaskStartScheduler()", main)
        self.assertIn("inputs.start_pressed = true;", safety_runtime)
        self.assertNotIn("inputs.start_pressed = line_start_ready;", safety_runtime)
        self.assertNotIn("line_start_ready = LineStartGate_Update", tasks + safety_runtime)

    def test_safety_task_arms_motor_only_after_all_tasks_are_online(self):
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        safety_runtime = (
            ROOT / "app/safety/safety_runtime.c"
        ).read_text(encoding="utf-8")

        control_start = tasks.index("static void ControlTask")
        safety_start = tasks.index("static void SafetyTask")
        display_start = tasks.index("static void DisplayTask")
        control_body = tasks[control_start:safety_start]
        safety_body = tasks[safety_start:display_start]

        self.assertNotIn("Motor_Safety_Arm()", control_body)
        self.assertNotIn("LineStartGate_Reset();", tasks)
        self.assertNotIn("line_start_ready", tasks)
        self.assertIn("SafetyRuntime_Step", safety_body)
        self.assertIn("AppBoot_IsMotorConfigured()", safety_runtime)
        arm_position = safety_runtime.index("Motor_Safety_Arm();")
        loop_position = safety_runtime.index("static void ArmWhenReady")
        self.assertGreater(arm_position, loop_position)
        self.assertIn(
            "BootTrace_AllTasksOnline()",
            safety_runtime[arm_position - 240:arm_position],
        )
        self.assertNotIn(
            "state == SAFETY_RUNNING",
            safety_runtime[arm_position - 120:arm_position + 40],
        )

    def test_safety_task_is_the_only_production_motor_arm_caller(self):
        callers = []
        definition = ROOT / "modules/motor/safety/motor_safety.c"

        for source in ROOT.rglob("*.c"):
            if source == definition:
                continue
            if "Motor_Safety_Arm(" in source.read_text(encoding="utf-8"):
                callers.append(source.relative_to(ROOT).as_posix())

        self.assertEqual(callers, ["app/safety/safety_runtime.c"])

    def test_motor_arm_call_is_unique_and_inside_safety_task(self):
        safety_runtime = (
            ROOT / "app/safety/safety_runtime.c"
        ).read_text(encoding="utf-8")
        arm_call = safety_runtime.index("Motor_Safety_Arm()")
        safety_start = safety_runtime.index("void SafetyRuntime_Step")
        helper_start = safety_runtime.index("static void ArmWhenReady")

        self.assertEqual(safety_runtime.count("Motor_Safety_Arm()"), 1)
        self.assertGreater(arm_call, helper_start)
        self.assertLess(helper_start, safety_start)

    def test_cm0_handlers_are_bound_for_ti_arm_clang(self):
        config = (ROOT / "FreeRTOSConfig.h").read_text(encoding="utf-8")

        for definition in (
            "#define xPortPendSVHandler PendSV_Handler",
            "#define vPortSVCHandler SVC_Handler",
            "#define xPortSysTickHandler SysTick_Handler",
        ):
            self.assertIn(definition, config)
        self.assertNotIn("#ifndef __TI_COMPILER_VERSION__", config)

    def test_kernel_build_shares_static_only_configuration(self):
        kernel = REPOSITORY / "freertos_kernel"
        project = (kernel / "freertos_kernel_ticlang.projectspec").read_text(
            encoding="utf-8"
        )
        makefile = (kernel / "Makefile").read_text(encoding="utf-8")
        car_project = (ROOT / ".project").read_text(encoding="utf-8")
        car_cproject = (ROOT / ".cproject").read_text(encoding="utf-8")

        self.assertEqual(
            list(REPOSITORY.rglob("FreeRTOSConfig.h")),
            [ROOT / "FreeRTOSConfig.h"],
        )
        self.assertIn("MSPM0G3507_LineFollowing_Car/FreeRTOSConfig.h", project)
        self.assertIn("CONFIG_DIR := ../MSPM0G3507_LineFollowing_Car", makefile)
        self.assertIn("$(CONFIG_DIR)/FreeRTOSConfig.h", makefile)
        for source in (project, makefile):
            self.assertIn("tasks.c", source)
            self.assertIn("list.c", source)
            self.assertIn("port.c", source)
            self.assertIn("portasm_diagnostic.c", source)
            self.assertNotIn("heap_", source)
            self.assertNotIn("timers.c", source)

        self.assertIn("freertos_kernel_ticlang", car_project)
        self.assertIn("freertos_kernel_ticlang.lib", car_cproject)
        self.assertIn("${WORKSPACE_LOC}/freertos_kernel_ticlang/Debug", car_cproject)
        self.assertNotIn("freertos_builds_LP_MSPM0G3507_release_ticlang", car_project)
        self.assertNotIn("freertos_builds_LP_MSPM0G3507_release_ticlang", car_cproject)

    def test_cli_and_ccs_share_the_local_diagnostic_portasm_wrapper(self):
        kernel = REPOSITORY / "freertos_kernel"
        project_path = kernel / "freertos_kernel_ticlang.projectspec"
        project = project_path.read_text(encoding="utf-8")
        makefile = (kernel / "Makefile").read_text(encoding="utf-8")

        ET.parse(project_path)
        self.assertIn("portasm_diagnostic.c", makefile)
        self.assertIn("portasm_diagnostic.c", project)
        self.assertNotIn(
            "portable/TI_ARM_CLANG/ARM_CM0/portasm.c\"",
            project,
        )
        port_rule = makefile[
            makefile.index("$(BUILD_DIR)/port.o:"):
            makefile.index("$(BUILD_DIR)/portasm.o:")
        ]
        self.assertNotIn("SVC_Handler", port_rule)
        self.assertNotIn("vRestoreContextOfFirstTask", port_rule)


if __name__ == "__main__":
    unittest.main()
