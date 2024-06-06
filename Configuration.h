// Configuration.h
// Este archivo contiene un montón de constantes y cosas que necesitas ajustar


// Paso 1: Invertir lo necesario. Cuando ejeecutas
// setAngles(45,45), los dos servos deberían mover hacía afuera.
// setAngles(90, 90), deberían mover hacía adentro.
// Si no lo hacen, invierte el variable apropiado.
#define INVERT_LEFT false
#define INVERT_RIGHT true

// Paso 2: Probablemente no necesitas tocar esto, 
// pero si cuando ejecutas setAngles(90,0), y un 
// mano no es horizontal, cambia estos valores  para que sea horizontal.
#define L_SCALE 1.
#define R_SCALE 1.

// ---------- NO EDITAR DEBAJO DE ESTA LÍNEA ----------

// Ángulos de las manos
#define L_OFFSET 70
#define R_OFFSET -70

// Constantes de la geometría del robót (millimeters)
#define MOTOR_TO_ORIGIN 23.25
#define MOTOR_ARM_LEN 60
#define PEN_ARM_LEN 80


// Pocisiónes para levantar al marcador
#define PEN_UP 45
#define PEN_DOWN 0