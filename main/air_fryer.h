#ifndef AIR_FRYER_H
#define AIR_FRYER_H

// Structure de l'état de l'airfryer
typedef struct {
    int temp;             // Température (ex: 400)
    int total_minutes;    // Temps total en minutes
    char time_display[16]; // Format d'affichage ("MM:SS" ou "HhMM")
    char state[8];        // "cooking", "paused", "stopped", "ON", "OFF"
    char mode[4];         // "AP" ou "STA"
} air_fryer_status_t;

// Déclaration externe de la variable globale
extern air_fryer_status_t fryer;

// Macro utilitaire pour les paramètres non utilisés
#define UNUSED_PARAMETER(x) (void)(x)

#endif // AIR_FRYER_H 