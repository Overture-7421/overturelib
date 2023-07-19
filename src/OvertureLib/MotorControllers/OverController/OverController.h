#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
/**
 * @brief The neutral mode determines how the motor controller behaves when not receiving any singnal.
 **/
template <class T>
class OverController {
public:

   /**
   * @brief Constructor de Overcontroller
   * @param _motorController Controlador de motor principal
   * @param _neutralMode Modo neutral del robot
   * @param _inverted Si el motor esta invertido
   * @param _gearRatio Relacion de engranajes del motor
   **/
   OverController(T* _motorController, ControllerNeutralMode _neutralMode, bool _inverted, double _gearRatio):
      motorController(_motorController),
      neutralMode(_neutralMode),
      inverted(_inverted),
      gearRatio(_gearRatio) {}

   /**
   * @brief regresa el controlador del motor principal
   * @return controlador del motor principal
   */
   T* getMotorController() {
      return motorController;
   }

   /**
   * @brief Inicializa el controlador de motor
   * (es para poner atributos dependiendo del objeto sin necesidad de cambiar tanto(template))
   * se utiliza en las clases de cada controlador del motor
   * se define en cada clase de controlador de motor
   */
   virtual void init() = 0;

   /**
   * @brief resetea el controlador de motor
   *
   * Se utiliza en las clases de cada controlador de motor
   * Se define en cada clase de controlador de motor
   */
   virtual void reset() = 0;

   /**
   * @brief Cambia el modo neutral del controlador de motor
   * @param _neutralMode Modo neutral del controlador de motor
   *
   * Se utiliza en clases de cada controlador de motor
   * Se define en cada clase de controlador de motor
   */
   virtual void setNeutralMode(ControllerNeutralMode _neutralMode) = 0;

   /**
   * @brief obtener la distancia recorrida del controlador de motor
   * @param _wheelDiameter Diametro de la llanta del robot
   * @return Distancia recorrida del controlador de motor en metros
   *
   * Se utiliza en las clases de cada controlador de motor
   * Se define en cada clase de controlador de motor
   */
   virtual double getDistance(double _wheelDiameter) = 0;

   /**
   * @brief obtener la velocidad del controlador de motor
   * @param _wheelDiameter Diametro de la llanta del robot
   * @return Velocidad  del controlador de motor en metros por segundo
   *
   * Se utiliza en las clases de cada controlador de motor
   * Se define en cada clase de controlador de motor
   */
   virtual double getVelocity(double _wheelDiameter) = 0;

   /**
   * @brief Se definen los valores de PIDS del controlador de motor
   * @param _p Valor de P
   * @param _i valor de I
   * @param _d valor de D
   * @param _s valor de S
   * @param _v valor de V
   * @param _slot Slot del controlador de motor
   *
   * Se utiliza en las clases de cada controlador de motor
   * Se define en cada clase de controlador de motor
   */
   virtual void setPIDValues(double _kP, double _kI, double _kD, double _kS, double _kV) = 0;

protected:
   T* motorController;
   ControllerNeutralMode neutralMode;
   bool inverted;
   double gearRatio;
};
