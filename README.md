#include <iostream>
#include <vector>
#include <cmath>

class Moteur {
public:
    float puissance;  // Puissance du moteur (en W)
    bool fonctionnel; // État du moteur : vrai s'il fonctionne, faux sinon


    Moteur(float p, bool f) : puissance(p), fonctionnel(f) {}


    float calculer_traction(float vitesse) {  // Traction/Poussée générée XXXXXX
        if (!fonctionnel) return 0.0f;  // Si le moteur est défaillant, pas de traction
        return puissance / vitesse;  // Traction/poussée simplifiée
    }
};

class Aerodynamique {
public:
    float C_L;   // Coefficient de portance
    float C_D;   // Coefficient de traînée
    float C_l;   // Coefficient de moment de roulis
    float C_m;   // Coefficient de moment de tangage
    float C_n;   // Coefficient de moment de lacet
    float S;     // Surface alaire de l'avion (en m²)
    float rho;   // Densité de l'air (en kg/m³)


    Aerodynamique(float cl, float cd, float cl_r, float cm, float cn, float s, float air_density)
        : C_L(cl), C_D(cd), C_l(cl_r), C_m(cm), C_n(cn), S(s), rho(air_density) {}

    
    float calculer_portance(float vitesse, float alpha) {       // Modèle simplifié (alpha=AoA) XXXXXXX
        return 0.5f * rho * std::pow(vitesse, 2) * S * C_L * std::cos(alpha); // Portance
    }

    float calculer_trainee(float vitesse, float alpha) {
        return 0.5f * rho * std::pow(vitesse, 2) * S * C_D * std::sin(alpha); // Traînée
    }

    float calculer_moment_roll(float delta_aileron, float vitesse) {      // Moment de roulis généré par les ailerons
        return 0.5f * rho * std::pow(vitesse, 2) * S * C_l * delta_aileron;
    }

    float calculer_moment_pitch(float delta_profondeur, float vitesse) {       // Moment de tangage généré par la profondeur
        return 0.5f * rho * std::pow(vitesse, 2) * S * C_m * delta_profondeur;
    }

    float calculer_moment_yaw(float delta_gouvernail, float vitesse) {        // Moment de lacet généré par le gouvernail
        return 0.5f * rho * std::pow(vitesse, 2) * S * C_n * delta_gouvernail;
    }
};

class Manoeuvre {
public:
    float delta_aileron;    // Angle de déviation de l'aileron
    float delta_gouvernail; // Angle du gouvernail
    float delta_profondeur; // Angle de profondeur

    Manoeuvre(float aileron, float gouvernail, float profondeur)
        : delta_aileron(aileron), delta_gouvernail(gouvernail), delta_profondeur(profondeur) {}

    
    void appliquer_manoeuvre(Aerodynamique& aero, float vitesse, float& M_roll, float& M_pitch, float& M_yaw) {
        M_roll += aero.calculer_moment_roll(delta_aileron, vitesse);     // Moment de roulis
        M_pitch += aero.calculer_moment_pitch(delta_profondeur, vitesse); // Moment de tangage
        M_yaw += aero.calculer_moment_yaw(delta_gouvernail, vitesse);    // Moment de lacet
    }
};

class Avion {
public:
    float x, y, z;          // Position dans l'espace (en m)
    float vitesse_x, vitesse_y, vitesse_z; // Vitesse (en m/s)
    float roll, pitch, yaw; // Orientation (en radians)
    float masse;            // Masse de l'avion (en kg)
    Moteur moteur;          // Moteur de l'avion
    Aerodynamique aero;     // Modèle aérodynamique
    Manoeuvre manoeuvre;     // Manœuvre appliquée à l'avion

    float M_roll, M_pitch, M_yaw;  // Moments de roulis, tangage, lacet

    Avion(float m, Moteur& mtr, Aerodynamique& aero, Manoeuvre& man)
        : masse(m), moteur(mtr), aero(aero), manoeuvre(man), x(0), y(0), z(0),
          vitesse_x(0), vitesse_y(0), vitesse_z(0), roll(0), pitch(0), yaw(0), M_roll(0), M_pitch(0), M_yaw(0) {}

    void initialiser() {
        x = 0; y = 0; z = 10000;
        vitesse_x = 250; vitesse_y = 0; vitesse_z = -100;
        roll = 0; pitch = 0; yaw = 0;
    }

    void calculer_forces() {
        float vitesse = std::sqrt(vitesse_x * vitesse_x + vitesse_y * vitesse_y + vitesse_z * vitesse_z);
        float portance = aero.calculer_portance(vitesse, pitch);  // alpha est le pitch pour simplification
        float trainee = aero.calculer_trainee(vitesse, pitch);
        float traction = moteur.calculer_traction(vitesse);

        std::cout << "Portance: " << portance << " N" << std::endl;
        std::cout << "Trainee: " << trainee << " N" << std::endl;
        std::cout << "Traction: " << traction << " N" << std::endl;
    }

    void calculer_moments() {
        float M_roll = 0, M_pitch = 0, M_yaw = 0;  // moments (roulis, tangage, lacet)
        float vitesse = std::sqrt(vitesse_x * vitesse_x + vitesse_y * vitesse_y + vitesse_z * vitesse_z);
        manoeuvre.appliquer_manoeuvre(aero, vitesse, M_roll, M_pitch, M_yaw);
        
        std::cout << "Moment de roulis: " << M_roll << " Nm" << std::endl;
        std::cout << "Moment de tangage: " << M_pitch << " Nm" << std::endl;
        std::cout << "Moment de lacet: " << M_yaw << " Nm" << std::endl;
    }

    void mettre_a_jour_position(float dt) {   // Méthode d'Euler
        float vitesse = std::sqrt(vitesse_x * vitesse_x + vitesse_y * vitesse_y + vitesse_z * vitesse_z);
    
        float portance = aero.calculer_portance(vitesse, pitch);  // Portance en fonction du pitch (angle d'attaque)
        float trainee = aero.calculer_trainee(vitesse, pitch);  // Traînée
        float traction = moteur.calculer_traction(vitesse);  // Traction du moteur (XXXXX : à orienter selon l'orientation de l'avion...?)
        float F_po = masse * 9.81f;  // Poids de l'avion (N)
    
        // Calcul des composantes de force selon les axes locaux (x, y, z) en fonction de l'orientation de l'avion
        float Fx = traction - trainee * std::cos(pitch);  // Force horizontale en X
        float Fy = -trainee * std::sin(pitch);            // Force horizontale en Y
        float Fz = portance - F_po - trainee * std::sin(pitch);  // Force verticale

        float accel_x = Fx / masse;  // Accélération dans la direction X
        float accel_y = Fy / masse;  // Accélération dans la direction Y
        float accel_z = Fz / masse;  // Accélération dans la direction Z

        vitesse_x += accel_x * dt;
        vitesse_y += accel_y * dt;
        vitesse_z += accel_z * dt;

        x += vitesse_x * dt;
        y += vitesse_y * dt;
        z += vitesse_z * dt;

        std::cout << "Nouvelle position: (" << x << ", " << y << ", " << z << ")" << std::endl;
        std::cout << "Nouvelle vitesse: (" << vitesse_x << ", " << vitesse_y << ", " << vitesse_z << ")" << std::endl;
    }

    void mettre_a_jour_orientation(float dt) {
    // Moments d'inertie de l'avion (simplifiés pour l'exemple)
    float I_x = 10000;  // Moment d'inertie autour de l'axe de roulis (en kg.m²)  (XXXX : notation x,y,z trompeuse)
    float I_y = 10000;  // Moment d'inertie autour de l'axe de tangage (en kg.m²)
    float I_z = 10000;  // Moment d'inertie autour de l'axe de lacet (en kg.m²)

    float accel_roll = M_roll / I_x;    // Accélération angulaire de roulis     // Récuperer les Moments dans le code XXXXXXXXXXXXXXX
    float accel_pitch = M_pitch / I_y;  // Accélération angulaire de tangage
    float accel_yaw = M_yaw / I_z;      // Accélération angulaire de lacet

    float omega_roll = accel_roll * dt;    // Vitesse angulaire de roulis
    float omega_pitch = accel_pitch * dt;  // Vitesse angulaire de tangage
    float omega_yaw = accel_yaw * dt;      // Vitesse angulaire de lacet

    roll += omega_roll * dt;
    pitch += omega_pitch * dt;
    yaw += omega_yaw * dt;

    std::cout << "Nouvelle orientation: (" << roll << ", " << pitch << ", " << yaw << ")" << std::endl;
    std::cout << "Vitesses angulaires: (" << omega_roll << ", " << omega_pitch << ", " << omega_yaw << ")" << std::endl;
}

    void simuler(float dt) {   // Fonction de simulation principale
        calculer_forces();
        calculer_moments();
        mettre_a_jour_position(dt);
        mettre_a_jour_orientation(dt);
    }
};

int main() {
    Moteur moteur(10000, true);  // Puissance du moteur et moteur fonctionnel
    Aerodynamique aero(0.5, 0.02, 0.1, 0.01, 0.01, 200, 1.225);  // Coefficients aérodynamiques et densité de l'air
    Manoeuvre manoeuvre(0.1, 0.1, 0.1);  // Angles de gouverne initiaux

    Avion avion(50000, moteur, aero, manoeuvre); // Masse de l'avion = 50 000 kg
    avion.initialiser();

    // Simulation pour 10 secondes avec un pas de temps de 1 seconde
    for (int i = 0; i < 10; ++i) {
        avion.simuler(1.0f);
    }

    return 0;
}
