#include <iostream>
#include <vector>
#include <cmath>
#include "LectureNACA.cpp"
#include <fstream>

const double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
const double g = 9.81;

class Aerodynamique {
public:
    float C_L = 0.0f;   // Coefficient de portance
    float C_D = 0.0f;   // Coefficient de traînée
    float C_m = 0.0f;   // Coefficient de moment de tangage
    float S;     // Surface alaire de l'avion (en m²)
    float rho;   // Densité de l'air (en kg/m³)
    float delta_profondeur = -0.110398;   // Choisi à alpha=1.5 deg UTILISER LE CODE EN COMMENTAIRE CI DESSOUS POUR LE RECALCULER
    Aerodynamics* polar; // pointeur vers les polaires lues

    Aerodynamique(float s, float air_density, Aerodynamics* polar_ptr = nullptr) : S(s), rho(air_density), polar(polar_ptr) {}

    void update_from_polar(float alpha_rad, float delta_p) {
        if (!polar) return;
        double alpha_deg = alpha_rad * RAD_TO_DEG;
        // C_L = static_cast<float>(polar->getCl(alpha_deg));
        // C_D = static_cast<float>(polar->getCd(alpha_deg));
        // C_m = static_cast<float>(polar->getCm(alpha_deg));
        C_L = 5 * (alpha_rad - (-0.035)) + 0.44 * delta_p;
        C_D = 0.0175 + 0.055 * std::pow(C_L, 2);
        C_m = -0.1 -(alpha_rad- (-0.035)) -1.46 * delta_p;
    }

    float calculer_portance(float vitesse) {
        return 0.5f * rho * std::pow(vitesse, 2) * S * C_L;
    }
    float calculer_trainee(float vitesse) {return 0.5f * rho * std::pow(vitesse, 2) * S * C_D;}

    float calculer_moment_pitch(float vitesse, float alpha_rad, float delta_p) {
        float l = 6.6f; // corde moyenne
        update_from_polar(alpha_rad, delta_p);
        return 0.5f * rho * std::pow(vitesse, 2) * S * C_m * l; // moment de tangage, il y a aussi une contribution de la vitesse au Cm, à multiplier par cos(alpha)?
    }
};

class Avion {
public:
    // Variables de position, vitesse, orientation et masse
    float x, y, z;          // Position dans l'espace (en m)
    float vitesse_x, vitesse_y, vitesse_z; // Vitesse (en m/s)
    float roll, pitch, yaw; // Orientation (en radians)
    float masse;            // Masse de l'avion (en kg)
    Aerodynamique aero;     // Modèle aérodynamique
    // Variables pour les forces et moments
    float portance, trainee, traction, poids;
    float Fx, Fy, Fz;
    float M_pitch;  // Moment/vitesse angulaire de tangage

    Avion(float m, Aerodynamique& aero)
        : masse(m), aero(aero), x(0), y(0), z(0),
          vitesse_x(0), vitesse_y(0), vitesse_z(0), roll(0), pitch(0), yaw(0),
          portance(0), trainee(0), traction(0), poids(0), Fx(0), Fy(0), Fz(0), M_pitch(0) {}

    void initialiser() {    
        // Initialisation des paramètres de l'avion
        x = 0; y = 0; z = 11000;
        vitesse_x = 285; vitesse_y = 0; vitesse_z = 0;
        roll = 0; pitch = 1.5f/RAD_TO_DEG; yaw = 0;
    }

    void calculer_forces(float &portance, float &trainee, float &traction, float &poids) { 
        float vitesse = std::sqrt(vitesse_x * vitesse_x + vitesse_y * vitesse_y + vitesse_z * vitesse_z); 
        portance = aero.calculer_portance(vitesse); 
        trainee = aero.calculer_trainee(vitesse); 
        poids = masse * g; 
        
        float traction_max = 2 * aero.rho / 1.292f * 120000.0f; // 2 moteur de F1=rho/rho0 * F_max 
        float delta_x = trainee / traction_max; //puissance relative des moteurs, nécessaire pour contrebalancer la trainee en croisère 
        traction = traction_max * delta_x; 
        // traction = poids*trainee/portance; // Vrai uniquement en vol de croisière 
    }

    float omega_pitch = 0;
    void mettre_a_jour_etat(float dt) { // Méthode d'Euler 
        const float I_y = 9720000.0f; // moment d'inertie autour de l'axe de tangage 
        float L = 0.0f, D = 0.0f, T = 0.0f, W = 0.0f; 

        float vitesse = std::sqrt(vitesse_x * vitesse_x + vitesse_y * vitesse_y + vitesse_z * vitesse_z); 
        float gamma = std::atan2(vitesse_z, vitesse_x); 
        float alpha = pitch - gamma; 

        if (alpha*RAD_TO_DEG < -15.25 || alpha*RAD_TO_DEG > 18.0) { 
            std::cout << "⚠️ alpha hors domaine : " << alpha*RAD_TO_DEG << "°" << std::endl; 
        } 

        aero.update_from_polar(alpha, aero.delta_profondeur); 
        calculer_forces(L, D, T, W); 
        M_pitch = aero.calculer_moment_pitch(vitesse, alpha, aero.delta_profondeur); 

        float Fx = (T - D) * (std::cos(yaw) * std::cos(gamma)) + L * (std::sin(gamma) * std::cos(roll)); 
        float Fy = (T - D) * (std::sin(yaw) * std::cos(gamma)) + L * (std::sin(gamma) * std::sin(roll)); 
        float Fz = (T - D) * (std::sin(gamma)) + L * (std::cos(gamma)) - W;

        // Calcul des accélérations et mises à jour des vitesses et positions
        float accel_x = Fx / masse;  // Accélération dans la direction X
        float accel_y = Fy / masse;  // Accélération dans la direction Y
        float accel_z = Fz / masse;  // Accélération dans la direction Z

        vitesse_x += accel_x * dt;
        vitesse_y += accel_y * dt;
        vitesse_z += accel_z * dt;

        x += vitesse_x * dt;
        y += vitesse_y * dt;
        z += vitesse_z * dt;

        // Mise à jour de l'angle de tangage
        float accel_pitch = M_pitch / I_y;
        omega_pitch += accel_pitch * dt;
        pitch += omega_pitch * dt;

        std::cout << "alpha=" << alpha*RAD_TO_DEG <<", pitch=" << pitch*RAD_TO_DEG <<"\n";
        std::cout << "Forces: Portance=" << L << " N, Trainée=" << D << " N, Traction=" << T << " N, Poids=" << W << " N\n"; 
        std::cout << "Forces: Fx=" << Fx << " N, Fy=" << Fy << " N, Fz=" << Fz << " N\n"; 
        std::cout << "Moments: M_pitch=" << M_pitch << " N.m\n"; 
        std::cout << "Etat linéaire: pos=(" << x << ',' << y << ',' << z << "), vel=(" << vitesse_x << ',' << vitesse_y << ',' << vitesse_z << ")\n";

    }
};

int main() {
    Aerodynamics polar;
    if (!polar.loadPolarCSV("xf-naca23012-il-1000000.csv")) {
        std::cerr << "Warning: impossible de charger polar CSV — utilisation des valeurs par défaut." << std::endl;
    }

    Aerodynamique aero(361.6f, 0.3639f, &polar);
    Avion avion(140178.9f, aero);
    avion.initialiser();

    const float dt = 0.1f;
    const float total_time = 100.0f;
    const int steps = static_cast<int>(total_time / dt);

    const char* csv_filename = "simulation_full.csv";
    std::ofstream csv(csv_filename);
    if (!csv.is_open()) {
        std::cerr << "Impossible d'ouvrir " << csv_filename << " en écriture\n";
        return 1;
    }

    // En-tête CSV : ajoute ou supprime champs selon besoins
    csv << "time,"
        << "x,y,z,vx,vy,vz,"
        << "roll,pitch,yaw,"
        << "M_pitch,"
        << "Fx,Fy,Fz,portance,trainee,traction,"
        << "Cl,Cd,Cm,"
        << "speed,AoA_deg\n";

    for (int i = 0; i < steps; ++i) {
        float t = (i + 1) * dt;
        avion.mettre_a_jour_etat(dt); // simuler une étape

        // Enregistrer les valeurs calculées précédemment
        float speed = std::sqrt(avion.vitesse_x * avion.vitesse_x + avion.vitesse_y * avion.vitesse_y + avion.vitesse_z * avion.vitesse_z);
        float AoA_deg = static_cast<float>(avion.pitch * RAD_TO_DEG) - std::atan2(avion.vitesse_z, avion.vitesse_x);

        // Écriture des données dans le fichier CSV
        csv << t << ',' << avion.x << ',' << avion.y << ',' << avion.z << ',' << avion.vitesse_x << ',' << avion.vitesse_y << ',' << avion.vitesse_z 
            << ',' << avion.roll << ',' << avion.pitch << ',' << avion.yaw << ',' << avion.M_pitch << ',' << avion.Fx << ',' << avion.Fy << ',' 
            << avion.Fz << ',' << avion.portance << ',' << avion.trainee << ',' << avion.traction << ',' << avion.aero.C_L << ',' << avion.aero.C_D << ',' << avion.aero.C_m 
            << ',' << speed << ',' << AoA_deg << '\n';
    }

    csv.close();
    std::cout << "Simulation enregistrée dans : " << csv_filename << std::endl;
    return 0;
}












// #include <iostream>
// #include <cmath>
// #include <fstream>
// #include "LectureNACA.cpp"

// const double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
// const double g = 9.81;

// class Aerodynamique {
// public:
//     float C_L = 0.0f;   // Coefficient de portance
//     float C_D = 0.0f;   // Coefficient de traînée
//     float C_m = 0.0f;   // Coefficient de moment de tangage
//     float S;     // Surface alaire de l'avion (en m²)
//     float rho;   // Densité de l'air (en kg/m³)
//     float delta_profondeur = 0;
//     Aerodynamics* polar; // pointeur vers les polaires lues

//     Aerodynamique(float s, float air_density, Aerodynamics* polar_ptr = nullptr) : S(s), rho(air_density), polar(polar_ptr) {}

//     void update_from_polar(float alpha_rad, float delta_p) {
//         if (!polar) return;
//         double alpha_deg = alpha_rad * RAD_TO_DEG;
//         // C_L = static_cast<float>(polar->getCl(alpha_deg));
//         // C_D = static_cast<float>(polar->getCd(alpha_deg));
//         // C_m = static_cast<float>(polar->getCm(alpha_deg));
//         C_L = 5 * (alpha_rad - (-0.035)) + 0.44 * delta_p;
//         C_D = 0.0175 + 0.055 * std::pow(C_L, 2);
//         C_m = -0.1 -(alpha_rad- (-0.035)) -1.46 * delta_p;
//     }

//     float calculer_portance(float vitesse) {
//         return 0.5f * rho * std::pow(vitesse, 2) * S * C_L;
//     }
//     float calculer_trainee(float vitesse) { return 0.5f * rho * std::pow(vitesse, 2) * S * C_D; }

//     float calculer_moment_pitch(float vitesse, float alpha_rad, float delta_p) {
//         float l = 6.6f; // corde moyenne
//         float d = l / 0.33f; // distance empennage - aile
//         float Se = 0.255 * S; // surface empennage
//         update_from_polar(alpha_rad, delta_p);
//         return 0.5f * rho * std::pow(vitesse, 2) * S * C_m * l; // moment de tangage
//     }
// };

// class Avion {
// public:
//     float x, y, z;          // Position dans l'espace (en m)
//     float vitesse_x, vitesse_y, vitesse_z; // Vitesse (en m/s)
//     float roll, pitch, yaw; // Orientation (en radians)
//     float masse;            // Masse de l'avion (en kg)
//     Aerodynamique aero;     // Modèle aérodynamique

//     float M_pitch;  // Moment/vitesse angulaire de tangage

//     Avion(float m, Aerodynamique& aero)
//         : masse(m), aero(aero), x(0), y(0), z(0),
//           vitesse_x(0), vitesse_y(0), vitesse_z(0), roll(0), pitch(0), yaw(0), M_pitch(0) {}

//     void initialiser() {    //// ALPHA=2.25 deg (courbe rapport BEA)
//         x = 0; y = 0; z = 11000;
//         vitesse_x = 285; vitesse_y = 0; vitesse_z = 0;
//         roll = 0; pitch = 1.5f/RAD_TO_DEG; yaw = 0;
//     }

//     // Recherche du delta_profondeur tel que C_m soit le plus proche de zéro
//     float trouver_delta_profondeur(float vitesse, float epsilon = 1e-6f) {
//         // Liste de valeurs possibles pour delta_profondeur (par exemple, entre -1.0 et 1.0)
//         int n = 10000; // Nombre de valeurs à tester
//         float delta_p_min = -0.13f;  // Bornes de recherche pour delta_profondeur
//         float delta_p_max = -0.11f;   // Bornes de recherche pour delta_profondeur
//         float step = (delta_p_max - delta_p_min) / n;

//         // Variables pour stocker le delta_profondeur optimal et le C_m minimal
//         float best_delta_p = delta_p_min;
//         float min_C_m = std::numeric_limits<float>::infinity();  // Initialisation à une grande valeur

//         // Itérer sur la liste de valeurs possibles de delta_profondeur
//         for (int i = 0; i <= n; ++i) {
//             float delta_p = delta_p_min + i * step;

//             // Calculer C_m pour la valeur actuelle de delta_profondeur
//             aero.update_from_polar(pitch, delta_p);
//             float C_m_guess = aero.C_m;

//             // Si le C_m est plus proche de zéro, on garde cette valeur
//             if (std::fabs(C_m_guess) < std::fabs(min_C_m)) {
//                 min_C_m = C_m_guess;
//                 best_delta_p = delta_p;
//             }

//             // Affichage de la progression
//             std::cout << "delta_p = " << delta_p << " , C_m = " << C_m_guess << std::endl;

//             // Si la différence est inférieure à epsilon, on peut arrêter plus tôt
//             if (std::fabs(C_m_guess) < epsilon) {
//                 break;
//             }
//         }

//         return best_delta_p;
//     }
// };

// int main() {
//     // Charger les données de la polaire de l'avion
//     Aerodynamics polar;
//     if (!polar.loadPolarCSV("xf-naca23012-il-1000000.csv")) {
//         std::cerr << "Warning: impossible de charger polar CSV — utilisation des valeurs par défaut." << std::endl;
//     }

//     // Paramètres de l'avion
//     Aerodynamique aero(361.6f, 0.4135f, &polar);
//     Avion avion(140000.0f, aero);
//     avion.initialiser();  // Initialisation des conditions de vol

//     // Vitesse de croisière (en m/s)
//     float vitesse = 285.0f;

//     // Trouver delta_profondeur pour rendre C_m aussi proche de zéro que possible
//     float delta_p = avion.trouver_delta_profondeur(vitesse);

//     // Affichage du résultat
//     std::cout << "Le delta de profondeur qui rend C_m le plus proche de zéro est : " << delta_p << std::endl;

//     return 0;
// }











