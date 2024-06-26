#include <Arduino.h>
//#include <vector>
//#include <queue>
//#include <tuple>
//#include <algorithm>
#include <limits.h>

#define INT1 5
#define INT2 6
#define INT3 9
#define INT4 10
#define TRIGGER_PIN 3
#define ECHO_PIN 4

#define FORWARD 'A'
#define REVERSE 'R'
#define STOP 'S'
 
const int setPoint = 100; // Set point para control de distancia (en cm)
const float Kp = 0.6, Ki = 0.2, Kd = 0.1; // Constantes del PID
long distancia;
int speedMotor = 255;
bool turnLeft = true; // Variable para alternar entre girar a la izquierda y a la derecha

// Variables para el PID
float time_PID = 0; // Tiempo actual del PID
float integral = 0; // Componente integral del PID
float time_prev = -1e-6; // Tiempo previo para el cálculo del PID
float e_prev = 0; // Error previo para el cálculo del PID
float curSpeed = 0; // Velocidad actual ajustada por el PID
const float deltat = 0.1; // Intervalo de tiempo entre las iteraciones del PID

void setup() {
  Serial.begin(9600);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(INT1, OUTPUT);
  pinMode(INT2, OUTPUT);
  pinMode(INT3, OUTPUT);
  pinMode(INT4, OUTPUT);
}

void loop() {
  distancia = Ultrasonido(TRIGGER_PIN, ECHO_PIN); // Medir la distancia con el sensor ultrasónico
  Serial.println(distancia);

  time_PID += deltat; // Actualizar el tiempo del PID
  float pid_out = PID(Kp, Ki, Kd, setPoint, distancia); // Calcular la salida del PID
  time_prev = time_PID; // Actualizar el tiempo previo para la siguiente iteración del PID

  ajustarVelocidad(pid_out); // Ajustar la velocidad de los motores basado en la salida del PID

  if (distancia < 20) { // Si la distancia medida es menor que 20 cm
    if (turnLeft) {
      girarIzquierda(); // Girar a la izquierda
    } else {
      girarDerecha(); // Girar a la derecha
    }
    turnLeft = !turnLeft; // Alternar el valor de turnLeft
    delay(500); // Espera medio segundo para girar
  } else {
    avanzar(); // Avanzar si no hay obstáculos cercanos
  }

  delay(100); // Esperar 100 ms antes de la siguiente iteración
}

float PID(float Kp, float Ki, float Kd, float setpoint, float measurement) {
  float offset = 320; // Valor base para la salida del PID
  float e = setpoint - measurement; // Calcular el error actual

  // Componente Proporcional
  float P = Kp * e;

  // Componente Integral
  integral += Ki * e * (time_PID - time_prev);

  // Componente Derivativa
  float D = Kd * (e - e_prev) / (time_PID - time_prev);

  // Calcular la salida del PID
  float MV = offset + P + integral + D;

  // Actualizar los valores previos para la siguiente iteración
  e_prev = e;
  time_prev = time_PID;

  return MV; // Devolver la salida del PID
}

void ajustarVelocidad(float pid_out) {
  // Ajustar la velocidad de los motores basado en la salida del PID
  if (pid_out > -1000) {
    curSpeed = 255; // Velocidad alta
  } else if (pid_out > -2000) {
    curSpeed = 100; // Velocidad media
  } else {
    curSpeed = 50; // Velocidad baja
  }

  // Ajustar velocidad de los motores
  controlarMotores(FORWARD, curSpeed, FORWARD, curSpeed);
}

void avanzar() {
  controlarMotores(FORWARD, curSpeed, FORWARD, curSpeed); // Avanzar con la velocidad ajustada
}

void girarIzquierda() {
  controlarMotores(REVERSE, curSpeed, FORWARD, curSpeed); // Girar a la izquierda
}

void girarDerecha() {
  controlarMotores(FORWARD, curSpeed, REVERSE, curSpeed); // Girar a la derecha
}

void controlarMotores(char dirA, int velA, char dirB, int velB) {
  motorA(dirA, velA);
  motorB(dirB, velB);
}

void motorA(char d, int velocity) {
  if (d == FORWARD) {
    digitalWrite(INT1, LOW);
    analogWrite(INT2, velocity);
  } else if (d == REVERSE) {
    analogWrite(INT1, velocity);
    digitalWrite(INT2, LOW);
  } else {
    digitalWrite(INT1, LOW);
    digitalWrite(INT2, LOW);
  }
}

void motorB(char d, int velocity) {
  if (d == FORWARD) {
    digitalWrite(INT3, LOW);
    analogWrite(INT4, velocity);
  } else if (d == REVERSE) {
    analogWrite(INT3, velocity);
    digitalWrite(INT4, LOW);
  } else {
    digitalWrite(INT3, LOW);
    digitalWrite(INT4, LOW);
  }
}

long Ultrasonido(int trigger, int eco) {
  long duration; // tiempo que demora en llegar el eco
  long distance; // distancia en centimetros

  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  duration = pulseIn(eco, HIGH); // obtenemos el ancho del pulso
  distance = (duration * 0.0343) / 2; // convertimos el tiempo a distancia
  return distance;
}

// Estructura para los nodos del algoritmo A*
struct Node {
  int x, y;
  float g, h; // g: cost from start, h: heuristic cost to goal
  Node* parent;

  Node(int x, int y, float g = 0, float h = 0, Node* parent = nullptr) 
    : x(x), y(y), g(g), h(h), parent(parent) {}

  float f() const { return g + h; }

  bool operator>(const Node& other) const { return f() > other.f(); }
};

// Función heurística (distancia Manhattan)
float heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

// Algoritmo A*
/*std::vector<Node> aStar(int startX, int startY, int goalX, int goalY, const std::vector<std::vector<int>>& grid) {
  // Conjunto de nodos abiertos (nodos por explorar) usando una cola de prioridad
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
  // Conjunto de nodos cerrados (nodos ya explorados)
  std::vector<std::vector<bool>> closedSet(grid.size(), std::vector<bool>(grid[0].size(), false));

  // Añadir el nodo inicial al conjunto abierto
  openSet.emplace(startX, startY, 0, heuristic(startX, startY, goalX, goalY));

  // Mientras haya nodos en el conjunto abierto
  while (!openSet.empty()) {
    // Obtener el nodo con el menor costo estimado
    Node current = openSet.top();
    openSet.pop();

    // Si el nodo actual es el objetivo, reconstruir el camino
    if (current.x == goalX && current.y == goalY) {
      std::vector<Node> path;
      Node* p = &current;
      while (p) {
        path.push_back(*p); // Añadir nodo al camino
        p = p->parent; // Seguir hacia el nodo padre
      }
      std::reverse(path.begin(), path.end()); // Invertir el camino para que vaya desde el inicio al objetivo
      return path; // Devolver el camino encontrado
    }

    // Marcar el nodo actual como cerrado (explorado)
    closedSet[current.x][current.y] = true;

    // Definir las direcciones de movimiento (derecha, abajo, izquierda, arriba)
    const std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    for (const auto& dir : directions) {
      int nx = current.x + dir.first;
      int ny = current.y + dir.second;

      // Verificar si el nodo vecino es válido y no ha sido explorado
      if (nx >= 0 && ny >= 0 && nx < grid.size() && ny < grid[0].size() && grid[nx][ny] == 0 && !closedSet[nx][ny]) {
        float g = current.g + 1; // Costo desde el inicio hasta el nodo vecino
        float h = heuristic(nx, ny, goalX, goalY); // Costo heurístico desde el vecino hasta el objetivo
        // Añadir el nodo vecino al conjunto abierto
        openSet.emplace(nx, ny, g, h, new Node(current));
      }
    }
  }

  // Devolver un camino vacío si no se encuentra una ruta
  return {};
}*/