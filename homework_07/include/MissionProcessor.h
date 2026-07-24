#pragma once
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IBalisticSolver.h"
class MissionProcessor {
    private:
        IConfigLoader* m_loader;
        ITargetProvider* m_provider;
        IBalisticSolver* m_solver;

        int m_currentStep = 0;
        int m_totalStep = 60;
    
    public:
        //Конструктор(паттерн) приймаємо інтерфейси через вказівники
        MissionProcessor(IConfigLoader* loader, ITargetProvider* provider, IBalisticSolver* solver)
        : m_loader(loader), m_provider(provider), m_solver(solver), m_currentStep(0) {}

            void init(const char* configSourse) {
                m_currentStep = 0;
                if (m_loader) {
                    //Тут IConfigLoader зчитує config.json nf ammo.json
                }
            }
        
        
    //Метод hasNext перевіряє чи не закінчились кроки симуляції
    bool hasNext() {
        return m_currentStep < m_totalStep;

    }

    void step() {
        if (!hasNext()) return;
        m_currentStep++;
    }

    void reset() {
        m_currentStep = 0;
    }

        
};