#include "../test.h"

#include "numerix/math/angle.h"

#include <iomanip>
#include <iostream>

using namespace numerix;

// Exercises the deg-branch of Angle::operator=(double), the default and
// three-double TaitBryanAngles constructors and the const-setters that
// take a (double, AngleType).
int main()
{
    print_title("Angle - 01");

    std::cout << std::fixed << std::setprecision(3);

    // ---- Angle::operator=(double) when the angle is stored in degrees.
    Angle a_deg(45.0, AngleType::deg);
    a_deg = 90.0;
    std::cout << " a_deg deg = " << a_deg.deg() << std::endl;
    std::cout << " a_deg rad = " << a_deg.rad() << std::endl;

    // ---- Angle::operator=(Angle) copy-assignment between objects.
    Angle src(60.0, AngleType::deg);
    Angle dst(0.0, AngleType::rad);
    dst = src;
    std::cout << " dst.deg   = " << dst.deg() << std::endl;
    std::cout << " dst.rad   = " << dst.rad() << std::endl;

    // ---- TaitBryanAngles default + three-double constructors.
    TaitBryanAngles tb_default;
    std::cout << " tb_def.roll = " << tb_default.roll() << std::endl;

    TaitBryanAngles tb_three(10.0, 20.0, 30.0, AngleType::deg);
    std::cout << " tb3.r deg = " << tb_three.roll(AngleType::deg) << std::endl;
    std::cout << " tb3.p deg = " << tb_three.pitch(AngleType::deg) << std::endl;
    std::cout << " tb3.y deg = " << tb_three.yaw(AngleType::deg) << std::endl;

    // ---- The const setters use the (double, AngleType) overloads.
    tb_three.roll(0.5, AngleType::rad);
    tb_three.pitch(0.6, AngleType::rad);
    tb_three.yaw(0.7, AngleType::rad);
    std::cout << " tb3.r rad = " << tb_three.roll(AngleType::rad) << std::endl;
    std::cout << " tb3.p rad = " << tb_three.pitch(AngleType::rad) << std::endl;
    std::cout << " tb3.y rad = " << tb_three.yaw(AngleType::rad) << std::endl;

    return 0;
}
