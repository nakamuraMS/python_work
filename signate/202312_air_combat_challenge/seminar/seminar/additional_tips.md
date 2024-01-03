# 目次

独自クラスの作成に関するTips

1. [戦況に関する主要な情報の取得方法](#戦況に関する主要な情報の取得方法)
    1. [時刻に関する情報](#時刻に関する情報)
    1. [戦闘場面の設定(Ruler)に関する情報](#戦闘場面の設定(Ruler)に関する情報)
    1. [自陣営のobservablesの取得](#自陣営のobservablesの取得)
    1. [戦闘機や誘導弾の運動状態に関する情報及び座標変換](#戦闘機や誘導弾の運動状態に関する情報及び座標変換(取扱説明書4.2項))
    1. [味方の誘導弾に関する情報の取得](#味方の誘導弾に関する情報の取得)
    1. [相手側の戦闘機や誘導弾の航跡情報](#相手側の戦闘機や誘導弾の航跡情報(取扱説明書4.3項))
    1. [自機の射程情報や射撃可否に関する情報](#自機の射程情報や射撃可否に関する情報(取扱説明書4.5.4項))

1. [独自のRewardクラスの作成方法](#独自のRewardの作成方法(取扱説明書5.2項))
    1. [observablesを介さないメンバへの直接アクセス](#observablesを介さないメンバへの直接アクセス)
    1. [PhysicalAsset(戦闘機や誘導弾)の取得](#PhysicalAsset(戦闘機や誘導弾)の取得)
    1. [Agentの取得](#Agentの取得)
    1. [イベントハンドラの使用](#イベントハンドラの使用)
    1. [過去のエピソードに関する情報の使用](#過去のエピソードに関する情報の使用)

1. [独自のMatchMakerの作成・使用方法](#独自のMatchMakerの作成・使用方法(取扱説明書3.7項))
    1. [学習サンプルでの使用方法](#学習サンプルでの使用方法)
        1. [HandyRL版サンプルで使用する場合](#HandyRL版サンプルで使用する場合)
        1. [ray RLLibサンプルで使用する場合](#ray%20RLLibサンプルで使用する場合)
    1. [独自MatchMakerクラスのひな形](#独自MatchMakerクラスのひな形)
    1. [初期条件の変更](#初期条件の変更)
    1. [matchTypeの変更](#matchTypeの変更)
    1. [その他の戦闘場面の変更](#その他の戦闘場面の変更)

1. [作成した独自クラスとモデルの使用方法](#作成した独自クラスとモデルの使用方法)
    1. [クラスの登録](#クラスの登録)
    1. [モデルの登録](#モデルの登録)
    1. [学習時の有効化方法](#学習時の有効化方法)
    1. [C++で独自クラスを作成・使用・投稿する方法](#C++で独自クラスを作成・使用・投稿する方法)

シミュレータ実装上の仕様に関するTips

1. [C++版のjson(nl::json)をPython側で取り扱う方法](#C++版のjson(nl::json)をPython側で取り扱う方法)
    1. [Pythonのプリミティブ型への変換](#Pythonのプリミティブ型への変換)
    1. [nljson型のオブジェクト生成](#nljson型のオブジェクト生成)
    1. [nljson型のままPython側で操作する方法](#nljson型のままPython側で操作する方法)
    1. [C++側でのjson pointerの利用](#C++側でのjson%20pointerの利用)

1. [強参照と弱参照](#強参照と弱参照)
    1. [weak_ptrからshared_ptrへの変換(C++側)](#weak_ptrからshared_ptrへの変換(C++側))
    1. [weak_ptrからshared_ptrへの変換(Python側)](#weak_ptrからshared_ptrへの変換(Python側))

# 戦況に関する主要な情報の取得方法
基本的な要素を組み合わせたサンプルはR5{Py}AgentSample01{S|M}やR5{Py}RewardSample{01|02}として提供されているが、ここではいくつかの主要な情報について具体的な取得方法を解説する。

AgentとRewardでは取得可能な情報が異なる(Rewardの方が多くの情報を得られる)ものの、基本的なインターフェースは類似しているため、主にAgent側からの取得方法を解説する。Rewardの場合は自機(parent)というものが存在しないため、場に存在する全ての機体から所望の機体を選択する処理が必要であり、その方法は[PhysicalAsset(戦闘機や誘導弾)の取得](#PhysicalAsset(戦闘機や誘導弾)の取得)に示す通りである。機体を選択した後の操作はAgentと同様に行えるほか、observableを介さずに対応するメンバを直接参照することができる。

## 時刻に関する情報
現在の時刻やtickの間隔等の情報は、SimulationManagerから取得可能である。取扱説明書の4.6.5項の表4.6-6に記載されている関数を呼び出すことで対応する情報を取得可能である。例えば現在時刻を取得する場合には、以下のように取得できる。AgentでもRewardでも記述方法は同様である。
```python
now = self.manager.getTime()
```
```c++
double now = this->manager->getTime();
```

## 戦闘場面の設定(Ruler)に関する情報
Rulerに関してAgentから観測可能な情報は以下の通りである。
```python
observables={
    "maxTime": float, # 最大戦闘時間[s]
    "minTime": float, # 最小戦闘時間[s]。令和5年度の設定では不使用。
    "eastSider": eastSider, # 東側スタートの陣営名(原則"Blue")
    "westSider": westSider, # 西側スタートの陣営名(原則"Red")
    "dOut": dOut, # 東西方向の広さ[m] (中心から防衛ラインまでの長さ)
    "dLine": dLine, # 南北方向の広さ[m] (中心から戦域端までの長さ)
    "hLim": hLim, # 高度方向の基準範囲。あくまで参考としての値であり、何の判定にも使用されない。
    "distanceFromBase": distanceFromBase,# 各陣営の防衛ラインから基地までの距離。令和5年度の設定では不使用。
    "fuelMargin": fuelMargin, # 帰還可否判定に使用する消費燃料のマージン倍率。令和5年度の設定では不使用。
    "forwardAx": forwardAx,# 各陣営の「前方」を表す二次元単位ベクトル。西側は[0,1]、東側は[0,-1]である。
                            # 東西を問わず共通のobservation,actionを使用できるようにするために役立てることを想定したもの。
    "sideAx": list[float], # 各陣営の「右方」を表す二次元単位ベクトル。西側は[-1,0]、東側は[1,0]である。
                            # 東西を問わず共通のobservation,actionを使用できるようにするために役立てることを想定したもの。
    "endReason": str, #終了の理由を表す文字列。令和5年度の設定ではNOTYET, ELIMINATION, PENALTY, TIMEUPの4種類。
}
```

Python版Agentでの取得は
```python
rulerObs = self.manager.getRuler()().observables # getRulerの後の2つめの()は弱参照の強参照化を行っているもの。nljson型である。
westSider = rulerObs["westSider"]() #末尾の()はnljsonをプリミティブ型に変換するもの。
```
のように、C++版Agentでの取得は
```c
auto rulerObs = this->manager->getRuler().lock()->observables; //const nl::json&型である。
std::string westSider = rulerObs.at("westSider"); // []によるアクセスも可能
```
のように行う。Rulerへの参照は弱参照として得られるため、[強参照と弱参照](#強参照と弱参照)に示すように強参照化が必要である。

## 自陣営のobservablesの取得
Agent自身の操作対象として割り当てられた機体にはself.parentsまたはself.parentでアクセス可能である。parentでない味方機のobservablesはparentのobservablesのうち"/shared/fighter/"以下に、各機のfullNameをキーとしたdict(object)として格納されている。
また、生存していないparentのobservablesはほとんど空であり、生存状態をparent.isAlive()で判定したうえで読み取りを行う必要がある。
サンプルでは上記の判定を行った上で、R5{Py}AgentSample01{S|M}のextractObservables関数の先頭で全味方機のobservablesをメンバ変数ourObservablesに格納している。

自陣営のobservablesの内訳は、「基準モデル及びパラメータに関する資料」の表1.4-2、表1.5-1、表1.6-1〜3の通りである。また、値としてのobservables以外にも表1.4-3に示すparentのメンバ関数を利用可能であり、例えば[自機の射程情報や射撃可否に関する情報の取得](#自機の射程情報や射撃可否に関する情報(取扱説明書4.5.4項))が可能である。また、observablesはnl::json型であるため、その取り扱いについては[ここ](#C++版のjson(nl::json)をPython側で取り扱う方法)も参照されたい。

なお、self.parentsのキーは戦闘機側のfullNameではなくAgentにとってのport名である。そのため、"/shared/fighter/"から味方機の情報を取得する場合にはparent.getFullName()を呼んでfullNameを取得する与える必要がある。

```python
ourObservables=[]
for parent in self.parents.values():
    if parent.isAlive():
        firstAlive=parent # Agentが動作する際には少なくとも1つのparentが生存しているため、
                          # それを起点にparentでない味方機や非生存機のobservablesを収集していく。
        break
for parent in self.parents.values():
    if parent.isAlive():
        # 残存していればobservablesそのもの
        ourObservables.append(parent.observables)
    else:
        # 被撃墜or墜落済なら本体の更新は止まっているので残存している親が代理更新したものを取得(誘導弾情報のため)
        ourObservables.append(
            firstAlive.observables.at_p("/shared/fighter").at(parent.getFullName()))
```
```c
std::vector<nl::json> ourObservables;
std::shared_ptr<PhysicalAssetAccessor> firstAlive=nullptr;
for(auto&& [port, parent] :parents){
    if(parent->isAlive()){
        firstAlive=parent; // Agentが動作する際には少なくとも1つのparentが生存しているため、
                           // それを起点にparentでない味方機や非生存機のobservablesを収集していく。
        break;
    }
}
for(auto&& [port, parent] :parents){
    if(parent->isAlive()){
        // 残存していればobservablesそのもの
        ourObservables.push_back(parent->observables);
    }else{
        // 被撃墜or墜落済なら本体の更新は止まっているので残存している親が代理更新したものを取得(誘導弾情報のため)
        ourObservables.push_back(
            firstAlive->observables.at("/shared/fighter"_json_pointer).at(parent->getFullName()));
    }
}
```

## 戦闘機や誘導弾の運動状態に関する情報及び座標変換(取扱説明書4.2項)
味方の戦闘機や誘導弾の運動状態はMotionStateクラスとして観測可能である。Agentクラスで自機の運動状態を取得する場合は以下のようにparentのobservables(json形式)からMotionStateインスタンスを生成できる。MotionStateインスタンスとして扱うことで、座標変換用の関数が利用可能となる。
```python
parentObs = self.parent.observables # nljson型のread-onlyプロパティである。
myMotion = MotionState(parentObs["motion"]) # nljson型のまま与えることができる。
pos = myMotion.pos # 位置や速度等のメンバにもアクセス可能

# 座標変換
ex = myMotion.relBtoP(np.array([1., 0., 0.])) # 機体座標系(B)における+x軸方向(1,0,0)を親座標系(慣性座標系)(P)に変換
northB = myMotion.relPtoB(np.array([1., 0., 0.])) # 親座標系(慣性座標系)(P)における+x軸方向(1,0,0)を機体座標系(B)に変換
northH = myMotion.relPtoH(np.array([1., 0., 0.])) # 親座標系(慣性座標系)(P)における+x軸方向(1,0,0)を局所水平座標系(H)に変換
```
```c
nl::json parentObs = parent->observables.at("motion");
MotionState myMotion(parentObs);
Eigen::Vector3d pos = myMotion.pos // 位置や速度等のメンバにもアクセス可能

// 座標変換
Eigen::Vector3d ex = myMotion.relBtoP(Eigen::Vector3d(1., 0., 0.)); // 機体座標系(B)における+x軸方向(1,0,0)を親座標系(慣性座標系)(P)に変換
Eigen::Vector3d notrhB = myMotion.relPtoB(Eigen::Vector3d(1., 0., 0.)); // 親座標系(慣性座標系)(P)における+x軸方向(1,0,0)を機体座標系(B)に変換
Eigen::Vector3d notrhH = myMotion.relPtoH(Eigen::Vector3d(1., 0., 0.)); // 親座標系(慣性座標系)(P)における+x軸方向(1,0,0)を局所水平座標系(H)に変換
```

使用可能な座標系は以下の3種類であり、observablesは基本的に慣性座標系(P)で与えられる。水平方向の位置と鉛直方向の高度は分けて扱った方が適切な場面が多いため、慣性座標系(P)でない座標系を用いたい場合は機体座標系(B)よりも局所水平座標系(H)に変換して扱うことを推奨する。

* 慣性座標系(P): 北を+X、東を+Y、鉛直下向きを+Zとした右手座標系(NED座標系)
* 機体座標系(B): 自機正面を+X、自機右方を+Y、自機下方を+Zとした右手座標系
* 局所水平座標系(H): 自機正面の方位に対応する水平方向を+X軸、鉛直下向きを+Z軸として構成した右手座標系

### 陣営座標系
parentのobservablesから得られるMotionStateは各機の基準座標系であるが、陣営単位で共通の座標系が定義されていた方が扱いやすい場面も存在する。サンプルではTeamOriginというクラス内クラスとして、MotionStateと同様の座標変換インターフェースを持った簡易的なクラスを実装し、メンバ変数teamOriginとして生成している。

なお、サンプルにおける陣営座標系とは、慣性座標系を自陣側戦域端のライン中央が原点となるように平行移動させた後、戦域中央が+x方向となるようにz軸まわりに回転させた座標系を指すが、任意の座標系に変更しても差し支えない。また、陣営座標系ははじめから水平であるため、局所水平座標系(H)を別途定義することはしていない。
```python
front = self.teamOrigin.relPtoB(np.array([1., 0., 0.])) # 親座標系(慣性座標系)(P)における+x軸方向(1,0,0)を陣営座標系(B)に変換 
```

## 味方の誘導弾に関する情報の取得
味方の誘導弾に関する情報は、各parentのobservablesの"/weapon/missiles"以下にarray(list)として格納されている。
飛翔中でないものも得られるため、Observationとして利用する際には適宜判定を行って必要な値のみを使う必要がある。

サンプルではR5{Py}AgentSample01{S|M}のextractObservables関数において、飛翔中かつ射撃時刻が古いものから順に格納するようにソートしてメンバ変数mslsに格納している。

```python
parentObs = self.parent.observables
def launchedT(m): # 飛翔中かつ射撃時刻が古いものから順にソートするための関数
    return m["launchedT"]() if m["isAlive"]() and m["hasLaunched"]() else np.inf
msls=sorted([m for m in parentObs.at_p("/weapon/missiles")],key=launchedT)
mslMotion = MotionState(msls[0]["motion"]) # 誘導弾についても"motion"キーの中身からMotionStateを生成可能
```
```c
nl::json parentObs = parent->observables.at("motion");
std::vector<nl::json> msls;
for(auto&& msl:parentObs.at("/weapon/missiles"_json_pointer)){
    msls.push_back(msl);
}
std::sort(msls.begin(),msls.end(),
[](const nl::json& lhs,const nl::json& rhs){
    // 飛翔中かつ射撃時刻が古いものから順にソートするための関数
    double lhsT,rhsT;
    if(lhs.at("isAlive").get<bool>() && lhs.at("hasLaunched").get<bool>()){
        lhsT=lhs.at("launchedT").get<double>();
    }else{
        lhsT=std::numeric_limits<double>::infinity();
    }
    if(rhs.at("isAlive").get<bool>() && rhs.at("hasLaunched").get<bool>()){
        rhsT=rhs.at("launchedT").get<double>();
    }else{
        rhsT=std::numeric_limits<double>::infinity();
    }
    return lhsT<rhsT;
});

MotionState mslMotion(msls[0].at("motion")); // 誘導弾についても"motion"キーの中身からMotionStateを生成可能
```


## 相手側の戦闘機や誘導弾の航跡情報(取扱説明書4.3項)
相手側の戦闘機の航跡はTrack3Dクラスとして観測可能であり、各parentのobservablesの"/sensor/track"以下にarray(list)として格納されている。令和5年度の設定ではここに格納される情報は全生存中parentで同一であるため、一つのparentからの抽出でよい。

サンプルではR5{Py}AgentSample01{S|M}のextractObservables関数において、自陣営の機体に近いものから順にソートしたうえでメンバ変数lastTrackInfoに格納している。
```python
parentObs = self.parent.observables
lastTrackInfo = [Track3D(t) for t in parent.observables.at_p("/sensor/track")]
pos = lastTrackInfo[0].posI() # 慣性系での位置を取得
pos = lastTrackInfo[0].velI() # 慣性系での速度を取得
```
```c
nl::json parentObs = parent->observables.at("motion");
std::vector<Track3D> lastTrackInfo;
for(auto&& t:parentObs.at("/sensor/track"_json_pointer)){
    lastTrackInfo.push_back(t);
}
Eigen::Vector3d pos = lastTrackInfo[0].posI(); // 慣性系での位置を取得
Eigen::Vector3d vel = lastTrackInfo[0].velI(); // 慣性系での速度を取得
```

相手側の誘導弾の航跡はTrack2Dクラスとして観測可能であり、各parentのobservablesの"/sensor/mws/track"以下にarray(list)として格納されている。こちらは戦闘機の航跡とは異なり、各parent自身が検出したもののみが格納されている。

サンプルではR5{Py}AgentSample01{S|M}のextractObservables関数において、自機の正面方向に近いものから順にソートしたうえでローカル変数mwsに格納している。
```python
parentObs = self.parent.observables
mws = [Track2D(t) for t in parentObs.at_p("/sensor/mws/track")]
direction = mws[0].dirI() # 到来方向ベクトル(慣性系)を取得
```
```c
nl::json parentObs = parent->observables.at("motion");
std::vector<Track2D> mws;
for(auto&& m:parentObs.at("/sensor/mws/track"_json_pointer)){
    mws.push_back(m);
}
Eigen::Vector3d direction = mws[0].dirI(); // 慣性系での到来方向ベクトルを取得
```

## 自機の射程情報や射撃可否に関する情報(取扱説明書4.5.4項)
* 自機の射程情報は、parentのgetRmax関数によって得ることができる。必ずしも実際の位置や速度に限定されず、仮想的な値を与えて計算することができる。サンプルでは、相手が自機の方を向いた場合(calcRHead)と背を向けた場合(calcRTail)の形で計算する例を示している。
* 自機の射撃可否は、parentのisLaunchable関数とisLaunchableAt関数によって得ることができる。前者は目標を指定せず、残弾数の有無等による自機のそもそもの射撃可否を返す。後者はTrack3Dとして引数に与えた特定の目標に対する射撃可否(発射可能範囲内に存在するか否か、有効な航跡か否か)を返す。

# 独自のRewardの作成方法(取扱説明書5.2項)

## observablesを介さないメンバへの直接アクセス
RewardクラスではAgentと同様のobservables経由でのアクセスも可能であるが、manager.getXXX()で得られたオブジェクトから各メンバを直接読むことも可能である。
ただし、C++版では基底クラスのポインタとして得られるため、メンバを定義している派生クラスへのキャストが必要である。[強参照と弱参照](#weak_ptrからshared_ptrへの変換(C++側))に示すように、強参照化とキャストを同時に行うgetShared関数も提供されているので参考とされたい。例えばRulerの場合、以下のようにメンバ変数に直接アクセスできる。
```python
ruler = self.manager.getRuler()() # getRulerの後の2つめの()は弱参照の強参照化を行っているもの。
westSider = ruler.westSider # Rewardからはメンバ変数への直接アクセスも可能
```
```c
auto ruler = getShared<R5AirToAirCombatRuler01>(manager->getRuler()); //強参照化とキャストを同時に行う。
std::string westSider = ruler->westSider; // Rewardからはメンバ変数への直接アクセスも可能
```

## PhysicalAsset(戦闘機や誘導弾)の取得
SimulationManagerのgetAssetsまたはgetAsset関数を用いて場に存在する任意のPhysicalAsset(戦闘機や誘導弾等)を取得可能である。両関数の違いは取扱説明書の4.6.5項の表4.6-6にある通り、イテラブルを返すか特定の名称に対応した一つのオブジェクトを返すかである。
通常はイテラブルで取得してfor文を回し、そのクラスや生存状態などで計算要否を判定しつつ、状態量を取得して報酬値を加算していくことになると思われる。サンプルのR5{Py}RewardSample02クラスでは以下のように戦闘機と誘導弾の情報を報酬計算に用いている。

陣営単位の報酬(TeamReward)では陣営で報酬の配分先を判定し、Agent単位の報酬(AgentReward)では各AgentのparentのfullNameで報酬の配分先を判定することになると思われる。

取得した後は、[Agentの場合](#自陣営のobservablesの取得)と同じように

```python
def onInnerStepEnd(self):
    ...略...
		for asset in self.manager.getAssets(): # 全PhysicalAssetに対しイテレーション
			asset=asset() # 弱参照を強参照化
            #if asset.getTeam()=="Blue": # 陣営名で判定することも可能
			if(isinstance(asset,Missile)): # 強参照化すれば、isinstanceでクラスの判定が可能
                ...略...
			elif(isinstance(asset,Fighter)):
				if(asset.isAlive()): # isAliveで生存状態も取得可能
```
```c
void R5RewardSample02::onInnerStepEnd(){
    for(auto&& e:manager->getAssets()){ // 全PhysicalAssetに対しイテレーション
        //if(e.lock()->getTeam()=="Blue"){} // 陣営名で判定することも可能
        if(isinstance<Missile>(e)){ // C++側では弱参照のままでもクラス判定が可能
            auto m=getShared<Missile>(e); //弱参照を強参照化
            ...略...
        }else if(isinstance<Fighter>(e)){
            auto f=getShared<Fighter>(e); //弱参照を強参照化
            if(f->isAlive()){ // isAliveで生存状態も取得可能
                ...略...
            }
        }
    }
}
```

## Agentの取得
SimulationManagerのgetAgentsまたはgetAgent関数を用いて場に存在する任意のAgentを取得可能である。両関数の違いは取扱説明書の4.6.5項の表4.6-6にある通り、イテラブルを返すか特定の名称に対応した一つのオブジェクトを返すかである。
通常はイテラブルで取得してfor文を回し、そのクラスや生存状態などで計算要否を判定しつつ、状態量を取得して報酬値を加算していくことになると思われる。
また、Agentそのものの状態量に基づいて報酬を与えることはあまり想定されず、各Agentのparentsの状態量に基づいて報酬を与えることが多いと思われる。
```python
for agent in self.manager.getAgents(): # 全Agentに対しイテレーション
    agent=agent() # 弱参照を強参照化
    #if agent.getTeam()=="Blue": # 陣営名で判定することも可能
    if agent.isAlive(): # isAliveで生存状態も取得可能
        for p in agent.parents.values(): # 大抵の場合はparentの状態量に基づいて報酬を計算することになる
            f=self.manager.getAsset(p.getFullName()) # parentに対応するPhysicalAsset(Accessorではない)を取得
```
```c
for(auto&& e:manager->getAssets()){ // 全PhysicalAssetに対しイテレーション
    auto agent=getShared<Agent>(e); //弱参照を強参照化
    //if(agent->getTeam()=="Blue"){} // 陣営名で判定することも可能
    if(agent->isAlive()){} // isAliveで生存状態も取得可能
    for(auto& p:agent->parents){ // 大抵の場合はparentの状態量に基づいて報酬を計算することになる
        auto f=manager->getAsset<Fighter>(p.second->getFullName()); //parentに対応するPhysicalAsset(Accessorではない)を取得
        ...略...
    }
}
```

## イベントハンドラの使用
取扱説明書2.4項に示す通り、本シミュレータは非周期的なイベントを取り扱う機能を有している。
現在の設定では墜落と撃墜の発生を監視可能としており、サンプルのR5{Py}RewardSample02クラスではコールバック関数を以下のように登録している。
```python
def onEpisodeBegin(self):#初期化
    ...略...
    self.manager.addEventHandler("Crash", self.onCrash)  # 墜落監視用のコールバックを登録
    self.manager.addEventHandler("Hit", self.onHit)  # 撃墜監視用のコールバックを登録
    ...略...
def onCrash(self, args):
    """墜落の監視用コールバック
    """
    asset=PhysicalAsset.from_json_weakref(args)() # 墜落した戦闘機が与えられる
    ...略...
def onHit(self, args):
    """撃墜の監視用コールバック
    """
    wpn=PhysicalAsset.from_json_weakref(args["wpn"])() # 命中した誘導弾が与えられる
    tgt=PhysicalAsset.from_json_weakref(args["tgt"])() # 撃墜された戦闘機が与えられる
    ...略...
```
```c
void R5RewardSample02::onEpisodeBegin(){
    ...略...
    manager->addEventHandler("Crash",[&](const nl::json& args){
        this->R5RewardSample02::onCrash(args);
    }); // 墜落監視用のコールバックを登録
    manager->addEventHandler("Hit",[&](const nl::json& args){
        this->R5RewardSample02::onHit(args);
    }); // 撃墜監視用のコールバックを登録
    ...略...
}
void R5RewardSample02::onCrash(const nl::json& args){
    std::shared_ptr<PhysicalAsset> asset=args; // 墜落した戦闘機が与えられる
    ...略...
}
void R5RewardSample02::onHit(const nl::json& args){
    std::shared_ptr<PhysicalAsset> wpn=args.at("wpn"); // 命中した誘導弾が与えられる
    std::shared_ptr<PhysicalAsset> tgt=args.at("tgt"); // 撃墜された戦闘機が与えられる
    ...略...
}
```

## 過去のエピソードに関する情報の使用
現状の実装では、Rewardインスタンスはエピソードごとに再生成されるため、単体では過去のエピソードに関する情報を持ち越して使用することはできない。ただし、外部ファイルに一時保存するか、データ保持用のCallbackクラス(RulerとReward以外のCallbackは持ち越し可能)を別途用意してmanagerのgetCallback()を用いて該当するCallbackオブジェクトからデータ取得を行うことによって実現可能である。

# 独自のMatchMakerの作成・使用方法(取扱説明書3.7項)
独自のMatchMakerクラスを作成する場合には、試行錯誤の過程で頻繁な修正が行われることが想定されるため、モジュールとしてのインストールではなく、学習を実行するスクリプトからインポートできる場所に直接.pyファイルを配置することを推奨する。

## 学習サンプルでの使用方法
例えば、TwoTeamCombatMatchMakerを継承した独自のMyMatchMakerクラスを作成し、MyMatchMaker.pyとして保存する場合を考える。
### HandyRL版サンプルで使用する場合
main.pyでインポートし、custom_classesというdictに追加しておく。
```python
from MyMatchMaker import MyMatchMaker
custom_classes={
    ...
    "MyMatchMaker": MyMatchMaker,
    ...
}
```

R5_contest_sample_S.yaml等のtrain_argsに"match_maker_class"というキーがあるので、上記の独自クラス名に変更して実行すれば、独自のMatchMakerで学習を行うことができる。
```yaml
train_args:
    ...
    match_maker_class: MyMatchMaker
    ...
```

### ray RLLibサンプルで使用する場合
LearningSample.pyでインポートし、matchMakerClassesというdictに追加しておく。
```python
matchMakerClasses={
    ...
    "MyMatchMaker": MyMatchMaker,
    ...
}
```

APPO_sample_S.json等に"match_maker_class"というキーがあるので、上記の独自クラス名に変更して実行すれば、独自のMatchMakerで学習を行うことができる。
```json
{
    ...
    "match_maker_class": "MyMatchMaker",
    ...
}
```

## 独自MatchMakerクラスのひな形
最低限のカスタマイズを行うにはmakeNextMatchメソッドのみオーバーライドすればよい。また、もしyaml等から設定したいパラメータがある場合にはinitializeメソッドもオーバーライドし、self.configから値を読み出しておけばよい。HandyRL版サンプルの場合は"match_maker_args"、ray RLLib版サンプルの場合は"match_maker"キーに与えたものがself.configとして与えられる。

対戦カードの書式は

```python
from ASRCAISim1.addons.MatchMaker.TwoTeamCombatMatchMaker import TwoTeamCombatMatchMaker

class MyMatchMaker(TwoTeamCombatMatchMaker):
    def initialize(self, config):
        super().initialize(config)

        # 独自の設定パラメータを設ける場合はinitialize()でself.configを呼び出せばよい。
        self.original_parameter=self.config["original_parameter"]

    def makeNextMatch(self,matchType,worker_index):
        '''
        '''
        ret={
            "Blue": { # 陣営名はself.teams(デフォルトは["Blue","Red"])からも取得できるが、直接書いても問題はない。
                "Policy": "Learner", #AgentConfigDispatcher
                "Weight": -1,
                "Suffix": "",
            },
            "Red": {
                "Policy": "Initial",
                "Weight": -1,
                "Suffix": "",
            },
        }

        return ret
```

## 初期条件の変更
サンプルのTwoTeamCombatMatchMakerクラスでは評価環境と同一の点対称ランダム配置となるように実装されているが、もしこれをカスタマイズして学習したい場合は、BlueとRedそれぞれの"InitialState"として、NED座標系での位置("pos")、速度("vel")、進行方向の方位("heading")を指定する必要がある。

なお、もし何も指定しなかった場合は、戦闘場面に関するconfigファイル(R5_contest_mission_config.json)に記述されている初期条件が使用される。

```python
class MyMatchMaker(TwoTeamCombatMatchMaker):
    def makeNextMatch(self,matchType,worker_index):
        ...略...

        #Policyの選択に加えて、必要に応じて初期条件の設定も行う。TwoTeamCombatMatchMakerクラスの実装も参考にされたい。
        ret["Blue"]["InitialState"]=[
            {
                "pos": [-10000.0, 20000.0, -12000.0],
                "vel": 270.0,
                "heading": 270.0,

            } for i in range(2) #機数分指定する。同じでもよいし、異なってもよい。
        ]
        ret["Red"]["InitialState"]=[
            {
                "pos": [10000.0, -20000.0, -12000.0],
                "vel": 270.0,
                "heading": 90.0,

            } for i in range(2) #機数分指定する。同じでもよいし、異なってもよい。
        ]

        ...略...
```

## matchTypeの変更
サンプルで与えられるmatchTypeは文字列であり、以下の通りである。
* HandyRL版の場合、yamlでtrain_argsのnameとして与えた名前(サンプルでは"Learner")の末尾に、学習用エピソードか評価用エピソード家に応じて":g"又は":e"のいずれかが付加されたもの。
* ray RLLib版の場合はTrainer名であり、jsonで"trainers"に記述されたもの(サンプルでは"Learner")となる。

もしmatchTypeに他の情報を付与する等の変更を行うなどしてmatchTypeに応じた生成を行いたい場合は、HandyRL版の場合はサンプルディレクトリ中のhandyrl/train.pyで、ray RLLib版の場合はMatchMakerアドオンのMatchMakerOnRay.pyでそれぞれmakeNextMatchメソッドを実行しているため、その引数を生成している部分を書き換えればよい。

## その他の戦闘場面の変更
makeNextMatchが生成するmatchInfoで制御できる場面設定はサンプルで示されているものに必ずしも限定されない。
TwoTeamCombatMatchMaker.pyに実装されているmanagerConfigReplacerメソッドでSimulationManagerのconfigの書き換え方法を記述し、wrapEnvForTwoTeamCombatMatchMakingメソッドでmatchInfoによる場面変更機能を付加したgym.Envクラスを生成しているため、独自のmatchInfoの内容に応じてこれら二つのメソッドを改変することで、SimulationManagerのconfigとして表現可能な任意の場面設定を動的に変更することができる。


# 作成した独自クラスとモデルの使用方法
AgentやRewardクラスを自作した場合、Factoryへのクラス、モデルの登録(取扱説明書4.1項)を行うことで使用可能となる。

## クラスの登録
サンプルのAgent,RewardクラスはOriginalModelSampleの__init__.pyやMain.cppで登録が行われている。
```python
from ASRCAISim1.common import addPythonClass
addPythonClass("Agent", "MyAgentClass", MyAgentClass)
```
```c
#include <ASRCAISim1/Factrory.h>
FACTORY_ADD_CLASS(Agent,MyAgentClass) // グループ名とクラス名を与える
FACTORY_ADD_CLASS(Reward,MyRewardClass) // グループ名とクラス名を与える

//クラス名と別名で登録したい場合はマクロでなく以下のstatic関数を直接呼ぶ。
Factory::addClass("Agent","AnotherName",&MyAgentClass::create<typename MyAgentClass::Type>);
```

## モデルの登録

### (1) Factory.addDefaultModelを呼び出す

```python
from ASRCAISim1.libCore import Factory
Factory.addDefaultModel( #staticメソッドであるaddDefaultModelを使用する。
    "Agent", # グループ名。Rewardの場合は"Reward"
    "MyAgentModel", # モデル名
    {
        "class": "MyAgentClass", # クラス名
        "config": {}, # コンストラクタに与えられるmodelConfig
    }
)
# あるいは、json又はjsonファイルから複数のモデルを一括で登録することもできる。
json = {
    "Agent": { # グループ名
        "MyAgentModel": { # モデル名
            "class": "MyAgentClass", #クラス名
            "config": {} # コンストラクタに与えられるmodelConfig
        }
    },
    "Reward": { # グループ名
        "MyRewardModel": { # モデル名
            "class": "MyRewardClass", #クラス名
            "config": {} # コンストラクタに与えられるmodelConfig
        }
    }
}
Factory.addDefaultModelsFromJson(json) # jsonファイルの場合はパスを引数としてaddDefaultModelsFromJsonFromJsonFileを呼ぶ。
```

### (2) SimulationManagerに与えるコンフィグの"/Factory/XXXX"に記述しておく
SimulationManager側で自動登録させることも可能であり、学習サンプルにおいては例えばR5_contest_agent_ruler_reward_models.jsonの"/Factory/Agent"や"/Factory/Reward"に追記することでも登録可能である。
```json
{
    "Factory": {
        "Agent": { # グループ名
            "MyAgentModel": { # モデル名
                "class": "MyAgentClass", #クラス名
                "config": {} # コンストラクタに与えられるmodelConfig
            }
        },
        "Reward": { # グループ名
            "MyRewardModel": { # モデル名
                "class": "MyRewardClass", #クラス名
                "config": {} # コンストラクタに与えられるmodelConfig
            }
        }
    }
}
```

## 学習時の有効化方法
学習サンプルでAgentモデルの有効化を行うには、R5_contest_learning_config_{S|M}.jsonの"/Manager/AgentConfigDispatcher/Learner_e/model"に登録したモデル名を記述することで行う。
Rewardモデルの有効化はR5_contest_learning_config_{S|M}.jsonの"/Manager/Rewards"に追加することで行う。

## C++で独自クラスを作成・使用・投稿する方法
手元で試行錯誤する場合にはOriginalModelSampleモジュールに独自クラスを追加して```pip install .```で再インストールするのが最も簡便な方法である。サンプルのAgentやRewardクラスと同じようにヘッダファイルとソースファイルを追加し、Main.cppのエントリポイント関数でクラスのPython側への公開とFactoryへの登録が行われるようにすれば学習サンプルで使用可能となる。
ただし、評価環境においてはOriginalModelSampleの改変や独自モジュールのインストールはできないため、手元で改変してインストールした後に生成されるlibOriginalModelSample.soを投稿物のディレクトリに直接配置し、__init__.pyにおいて以下のようにsoファイルから直接該当クラスをインポートすればよい。
```python
def getUserAgentClass(args={}):
    from .libOriginalModelSample import MyGreatAgent
    return MyGreatAgent
```

なお、OriginalModelSampleや他の参加者と同一の名称のクラスを用いていた場合はFactoryへの登録に関する警告が出ることとなるが、MinimumEvaluationやvalidate.py、評価用サーバでは固有の名称で登録されることとなるため問題ない。

# C++版のjson(nl::json)をPython側で取り扱う方法
C++側においてjsonは取扱説明書1.2.2項(4)に示したnlohmann's jsonというライブラリが提供するnl::json型を用いて扱っている。これをPython側からアクセスする場合に最もシンプルな方法は、取扱説明書1.2.2項(5)に依存ライブラリとして挙げたpybind11_jsonのようにPythonのプリミティブ型(dict,list,str,float,int,bool,None)に自動変換するものである。しかし、このような場合、C++側で保持されているnl::json型変数の中身をPython側から書き換えるという操作ができないほか、PythonとC++を行き来する都度変換が行われてしまうため、Pythonのプリミティブ型への自動変換を無効化しMutableなnljsonクラスとしてPython側に公開している。

例えば、AgentやReward等のFactory経由で生成される登場物のコンストラクタ引数として与えられるmodelConfigとinstanceConfigは、dictではなくnljson型で与えられる。
また、Agentクラスで戦闘機側への制御出力を記述する先のメンバ変数であるcommandsやobservablesもC++の基底クラスのメンバでありnljson型である。

## Pythonのプリミティブ型への変換
nljson型をPythonのプリミティブ側に変換する際には()を付けて__call__を呼べばよい。
```python
primitive = self.modelConfig()
```

## nljson型のオブジェクト生成
Python型からもnljson型のオブジェクトを生成することが可能である。
nljsonのコンストラクタ引数に任意のjson化可能なオブジェクトを与えるとそれに対応したnljson型のオブジェクトが生成される。
```python
from ASRCAISim1.libCore import nljson # from ASRCAISim1.libCore import *でも可
j = nljson({"a": 1, "b": "2", "c": [{"d": 3.0}, True, None]})
```

## nljson型のままPython側で操作する方法
nljson型のまま扱う場合、大抵の場合は中身がdict(object)かlist(array)であると想定される。in演算子やfor文の使用、[]による値の取得や設定はdictやarrayと同様のインターフェースで可能となっている。
ただし、dictのgetに相当する関数は未実装であり、in演算子による存在判定を行って手動で記述しなければならない。

```python
j = nljson({"a": 1, "b": "2", "c": [{"d": 3.0}, True, None]})
print("a" in j) #True
for k, v in j.items(): #keysやvaluesも可能。list(array)の場合もfor v in j:のように記述可能。
  print(k, ":", v)
j["e"] = "HOGE"

#value = j.get("f", "default value") # 未実装なので以下のようにする。
value = j["f"] if "f" in j else "default value"
```

また、nljson型特有の機能として、json pointerとmerge patchに対応している。前者は階層化されたjson中の要素を単一の"/"区切りの文字列で表現するための仕様であり、nljson.at_pを用いて値を取得できるようになっている。
```python
j = nljson({"a": 1, "b": "2", "c": [{"d": 3.0}, True, None]})
elem = j.at_p("/c/0/d") # 3.0
```

後者はいわゆる再帰型のdict.updateに類似した操作であり、元のjsonの一部を修正するためのものである。nljson.merge_patchを用いて値の修正が可能となっている。なお、merge patchの再帰はdict(object)のみに対して行われることとされているため、list(array)に対しては行われず、list(array)は配列全体の削除または置換となる。
```python
j = nljson({"a": 1, "b": {"ba": 2}, "c": [{"d": 3.0}, True, None]})
patch = nljson({"a": None, "b": {"bb": 4}, "c": [55]}) #patchでNone(null)を指定した部分は要素の削除となる。
j.merge_patch(patch)
j # {"b": {"ba": 2, "bb": 4}, "c": [55]}
```

## C++側でのjson pointerの利用
nlohmann's jsonではjson pointerは文字列リテラルによって表現されており、_json_pointerをサフィックスを付けることで使用可能となる。
```c
nl::json j = {{"a", 1}, {"b", "2"}, {"c", {{"d": 3.0}}, true, nullptr}};
nl::json elem = j.at("/c/0/d"_json_pointer); // 3.0
```


# 強参照と弱参照
本シミュレータは、登場物の循環参照を避けるために強参照(shared_ptr)と弱参照(weak_ptr)を使い分けて実装されている。
基本的には、シミュレーションの実行を管理するSimulationManagerクラスが各インスタンスのshared_ptrを保持し、それ以外のクラスでは自身が属するSimualtionManagerインスタンス経由で弱参照を取得して他のインスタンスのメンバにアクセスする際に一時的に強参照化して用いる仕様となっている。

そのため、独自クラスを作成する際に他の登場物への参照をメンバ変数として保持する際には、自身が新たに生成したインスタンスでない限り基本的には弱参照(weak_ptr)として保持することを推奨する。
例えばSimulationManagerのgetRuler、getAssets、getAgents等は弱参照を返すため、RewardやCallbackで利用したい場合はこれらの返り値をメンバとして保持するのがよい。
なお、Agentにとってのparentについては値の不正取得防止用の仲介用クラスであるAssetAccessor型となっており、その内部で弱参照の操作が行われているため、強参照化の処理を個別に行う必要はない。

## weak_ptrからshared_ptrへの変換(C++側)
C++の場合、weak_ptr::lock()で対応するshared_ptrを取得できる。
また、ユーティリティ関数としてUtility.hにgetSharedという関数が用意されており、shared_ptrかweak_ptrのいずれかを引数として与えて、テンプレート引数として中身のキャスト先派生クラスを指定することで所望の型のshared_ptrを得ることができるようになっている。
ただし、現時点の実装ではキャスト前後の型が一致している場合にはテンプレート引数を二つ重ねる必要がある。
```c
auto shared=getShared<Dst>(src); //srcの中身がDstでない(=Dstの基底クラス)場合
auto shared=getShared<Dst,Dst>(src); //srcの中身がDstである場合
auto shared=src.lock(); //srcがweak_ptrであることが分かっている場合、キャストが不要ならgetSharedを呼ばずに単にlockを呼んでもよい。
```

## weak_ptrからshared_ptrへの変換(Python側)
Python側には通常shared_ptrの形でC++側のオブジェクトが公開されるよう実装されているが、状況によってはweak_ptrを取り扱う必要がある。weak_ptrに対応するクラスとしてWeakrefForPYが実装されており、Pythonの標準の弱参照モジュールであるweakrefのweakref.refと同様に()を付けることで元のオブジェクト(shared_ptr)を取得可能としている。
```python
weak = self.manager.getRuler() #SimulationManagerのgetRulerは弱参照を返す。
ruler = weak() # ()で強参照(shared_ptr)に変換する。通常のPythonオブジェクトとしてメンバにアクセス可能となる。
               # このとき、self.ruler = weak() のように強参照のままメンバ変数として保持することは、循環参照が生じて
               # オブジェクトの破棄が行われなくなる可能性があるため推奨されない。
```
