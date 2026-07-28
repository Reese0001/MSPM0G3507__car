#include "motor_configuration.h"

#include "../../time/delay.h"

static bool SendAndWait(bool ok)
{
    if (!ok) {
        return false;
    }
    delay_ms(100);
    return true;
}

bool Set_Motor(int MOTOR_TYPE)
{
    bool ok = false;

    if(MOTOR_TYPE == 1)
    {
        ok = SendAndWait(send_motor_type(1));
        ok = ok && SendAndWait(send_pulse_phase(30));
        ok = ok && SendAndWait(send_pulse_line(11));
        ok = ok && SendAndWait(send_wheel_diameter(67.00f));
        ok = ok && SendAndWait(send_motor_deadzone(1900));
        return ok;
    }

    else if(MOTOR_TYPE == 2)
    {
        ok = SendAndWait(send_motor_type(2));
        ok = ok && SendAndWait(send_pulse_phase(20));
        ok = ok && SendAndWait(send_pulse_line(13));
        ok = ok && SendAndWait(send_wheel_diameter(48.00f));
        ok = ok && SendAndWait(send_motor_deadzone(1600));
        return ok;
    }

    else if(MOTOR_TYPE == 3)
    {
        ok = SendAndWait(send_motor_type(3));
        ok = ok && SendAndWait(send_pulse_phase(45));
        ok = ok && SendAndWait(send_pulse_line(13));
        ok = ok && SendAndWait(send_wheel_diameter(68.00f));
        ok = ok && SendAndWait(send_motor_deadzone(1600));
        return ok;
    }

    else if(MOTOR_TYPE == 4)
    {
        ok = SendAndWait(send_motor_type(4));
        ok = ok && SendAndWait(send_pulse_phase(48));
        ok = ok && SendAndWait(send_motor_deadzone(1000));
        return ok;
    }

    else if(MOTOR_TYPE == 5)
    {
        ok = send_motor_type(1);
        if (!ok) return false;
        delay_ms(100);
        ok = send_pulse_phase(40);
        if (!ok) return false;
        delay_ms(100);
        ok = send_pulse_line(11);
        if (!ok) return false;
        delay_ms(100);
        ok = send_wheel_diameter(67.00f);
        if (!ok) return false;
        delay_ms(100);
        ok = send_motor_deadzone(1900);
        if (!ok) return false;
        delay_ms(100);
        return true;
    }

    return false;
}
