// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#pragma once
#include <vector>
#include <map>
#include <pybind11/pybind11.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "MathUtility.h"
#include "Utility.h"
#include "Reward.h"
#include "R5AirToAirCombatRuler01.h"
namespace py=pybind11;
namespace nl=nlohmann;

DECLARE_CLASS_WITHOUT_TRAMPOLINE(R5AirToAirCombatBasicReward01,TeamReward)
    public:
    /*基本的な空対空戦闘のルールの得点要素に対応した基本報酬のクラス。
        実装上の仕様はR4年度コンテストで使用されたものと同一であり、パラメータ設定によってR5年度コンテストのルールに対応した報酬を表現できる。
        単に得点をそのまま報酬として扱いたい場合はScoreRewardクラスを使用してもよいが、必ずしも得点と報酬は連動すべきとは限らない。
        また、Rulerの一部の得点計算は戦闘終了時点まで保留されることがある一方で、報酬は遅延させずに与えたいということも想定される。
        そのため、陣営ごとの即時報酬として計算するクラスのひな形をサンプルとして実装するものである。
        なお、各得点増減要因の発生時の報酬を変化させたい場合は、modelConfigの以下のパラメータを設定することで実現可能である。（デフォルトは全て0またはtrueであり、得点の値と同等となる。）
        1. rElim：相手が全滅した際の追加報酬
        2. rElimE：自陣営が全滅した際の追加報酬
        3. rBreakRatio：相手の防衛ラインを突破した際の得点を報酬として与える際の倍率(1+rBreakRatio倍)
        4. rBreak：相手の防衛ラインを突破した際の追加報酬
        5. rBreakE：相手に防衛ラインを突破された際の追加報酬
        6. adjustBreakEnd (bool)：Rulerが進出度に応じた得点をしない条件で終了した際に進出度に応じた報酬を無効化するかどうか
        7. rTimeup：時間切れとなった際の追加報酬
        8. rDisq：自陣営がペナルティによる敗北条件を満たした際の追加報酬
        9. rDisqE：相手がペナルティによる敗北条件を満たした際の追加報酬
        10. rHitRatio：相手を撃墜した際の得点を報酬として与える際の倍率(1+rHitRatio倍)
        11. rHit：相手を撃墜した際の追加報酬
        12. rHitE：相手に自陣営の機体が撃墜された際の追加報酬
        13. rAdvRatio：進出度合いに応じた得点(の変化率)を報酬として与える際の倍率(1+rAdvRatio倍)
        14. acceptNegativeAdv：相手陣営の方が進出度合いが大きい時も負数として報酬化するかどうか(ゼロサム化を行う場合は意味なし)
        15. rCrashRatio：自陣営の機体が墜落した際の得点を報酬として与える際の倍率(1+rCrashRatio倍)
        16. rCrash：自陣営の機体が墜落した際の追加報酬
        17. rCrashE：相手が墜落した際の追加報酬
        18. rAliveRatio：自陣営の機体が得た生存点を報酬として与える際の倍率(1+rAliveRatio倍)
        19. rAlive: 自陣営の機体が生存点を得た際の追加報酬
        20. rAliveE：相手が生存点を得た際の追加報酬
        18. rOutRatio：場外ペナルティによる得点を報酬として与える際の倍率(1+rOutRatio倍)
        19. adjustZerosum：相手陣営の得点を減算してゼロサム化するかどうか
        20. rHitPerAircraft：R5AirToAirCombatRuler01のscorePerAircraftと同様の発想で、rHitの値を「1機あたりの増減量」と「陣営全体での総増減量」のどちらとして解釈するかのフラグ。
        21. rHitEPerAircraft：R5AirToAirCombatRuler01のscorePerAircraftと同様の発想で、rHitEの値を「1機あたりの増減量」と「陣営全体での総増減量」のどちらとして解釈するかのフラグ。
        22. rCrashPerAircraft：R5AirToAirCombatRuler01のscorePerAircraftと同様の発想で、rHitCrashの値を「1機あたりの増減量」と「陣営全体での総増減量」のどちらとして解釈するかのフラグ。
        23. rCrashEPerAircraft：R5AirToAirCombatRuler01のscorePerAircraftと同様の発想で、rHitCrashEの値を「1機あたりの増減量」と「陣営全体での総増減量」のどちらとして解釈するかのフラグ。
        20. rAlivePerAircraft：R5AirToAirCombatRuler01のscorePerAircraftと同様の発想で、rAliveの値を「1機あたりの増減量」と「陣営全体での総増減量」のどちらとして解釈するかのフラグ。
        21. rAliveEPerAircraft：R5AirToAirCombatRuler01のscorePerAircraftと同様の発想で、rAliveEの値を「1機あたりの増減量」と「陣営全体での総増減量」のどちらとして解釈するかのフラグ。
	*/
    bool debug;
    double rElim,rElimE,rBreakRatio,rBreak,rBreakE,rTimeup,rDisq,rDisqE,rHitRatio,rAdvRatio,rCrashRatio,rAliveRatio,rOutRatio;
    std::map<std::string,double> rHit,rHitE,rCrash,rCrashE,rAlive,rAliveE;
    std::map<std::string,std::map<std::string,double>> rHitScale,rHitEScale,rCrashScale,rCrashEScale,rAliveScale,rAliveEScale;
    bool rHitPerAircraft,rHitEPerAircraft,rCrashPerAircraft,rCrashEPerAircraft,rAlivePerAircraft,rAliveEPerAircraft;
    bool adjustBreakEnd,acceptNegativeAdv,adjustZerosum;
    std::shared_ptr<R5AirToAirCombatRuler01> ruler;
    std::map<std::string,std::string> opponentName;
    std::map<std::string,double> advPrev;
    std::map<std::string,double> advOffset;
    std::map<std::string,double> breakTime;
    std::map<std::string,double> disqTime;
    std::map<std::string,double> eliminatedTime;
    //constructors & destructor
    R5AirToAirCombatBasicReward01(const nl::json& modelConfig_,const nl::json& instanceConfig_);
    virtual ~R5AirToAirCombatBasicReward01();
    //functions
    void debugPrint(const std::string& reason,const std::string& team,double value);
    virtual void validate();
    virtual void onEpisodeBegin();
    virtual void onInnerStepEnd();
    virtual void onStepEnd();
    virtual double getRHit(const std::string& team,const std::string& modelName) const;
    virtual double getRHitE(const std::string& team,const std::string& modelName) const;
    virtual double getRCrash(const std::string& team,const std::string& modelName) const;
    virtual double getRCrashE(const std::string& team,const std::string& modelName) const;
    virtual double getRAlive(const std::string& team,const std::string& modelName) const;
    virtual double getRAliveE(const std::string& team,const std::string& modelName) const;
};
DECLARE_TRAMPOLINE(R5AirToAirCombatBasicReward01)
    virtual double getRHit(const std::string& team,const std::string& modelName) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,getRHit,team,modelName);
    }
    virtual double getRHitE(const std::string& team,const std::string& modelName) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,getRHitE,team,modelName);
    }
    virtual double getRCrash(const std::string& team,const std::string& modelName) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,getCrash,team,modelName);
    }
    virtual double getRCrashE(const std::string& team,const std::string& modelName) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,getCrashE,team,modelName);
    }
};

void exportR5AirToAirCombatBasicReward01(py::module& m);
