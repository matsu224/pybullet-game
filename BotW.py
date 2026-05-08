import pybullet as p
import pybullet_data
import time
from time import sleep

#迷路をランダム生成しないバージョンの完成形

def make_multi(dimlist,poslist,rgba,basepos):
    lmass=[] #0.(固定)
    collisionShapelist=[] #p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=dim[i])
    visualShapelist=[] #p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=dim[i])
    #lp=[] #pos[i]
    lo=[] #[0, 0, 0, 1](固定)
    #lifp=[] #pos[i]
    lifo=[] #[0, 0, 0, 1](固定)
    lpi=[] #0(固定)
    ljt=[] #p.JOINT_FIXED(固定)
    lja=[] #[0, 0, 0](固定)
    
    for i in range(0,len(dimlist)):
        lmass.append(0.)
        c=p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=dimlist[i])
        collisionShapelist.append(c)
        v=p.createVisualShape(shapeType=p.GEOM_BOX, rgbaColor=rgba, halfExtents=dimlist[i]) #色はintで指定するとうまくいかなかった、なぜ？
        visualShapelist.append(v)
        #lp.append(poslist[i])
        lo.append([0, 0, 0, 1])
        #lifp.append(poslist[i]) 
        lifo.append([0, 0, 0, 1])
        lpi.append(0) #ここがfloatになっていてエラーが起きていた
        ljt.append(p.JOINT_FIXED)
        lja.append([0, 0, 0])
    
    multi_body = p.createMultiBody( #マルチボディの作成
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=-1,
        basePosition=basepos, #基準位置
        baseOrientation=[0, 0, 0, 1], #???
        linkMasses=lmass, #それぞれの質量
        linkCollisionShapeIndices=collisionShapelist, #衝突判定用
        linkVisualShapeIndices=visualShapelist, #外見
        linkPositions=poslist, #親リンクからの子リンク原点の位置
        linkOrientations=lo, #親リンクからの子リンクの方向・回転
        linkInertialFramePositions=poslist, #???
        linkInertialFrameOrientations=lifo, #???
        linkParentIndices=lpi, #???
        linkJointTypes=ljt, #ジョイントの種類
        linkJointAxis=lja, #ジョイントの軸 #???
    )
    
    return multi_body


cid = p.connect(p.GUI) #GUIに接続
if (cid < 0): #接続が失敗した場合に再度接続
  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #pybullet_dataのサンプルのパスを通す？
p.setPhysicsEngineParameter(numSolverIterations=10) #最大反復回数、おそらく物理演算を何回で打ち切るか？
p.setTimeStep(1. / 240.) #1秒あたりの処理数？

#以下configureDebugVisualizerで機能の有効無効を設定
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0) # キーボードショートカットを無効にする

#カメラの初期化設定
camera=0
camera_distance = [5.0,5.0]
camera_yaw = [0,0] #水平方向の回転角度
camera_pitch = [-35,-85] #垂直方向の回転角度
camera_target_position = [[0, 0, 4],[-3.2,0,4]] #カメラのターゲットとなる位置
p.resetDebugVisualizerCamera(camera_distance[camera], camera_yaw[camera], camera_pitch[camera], camera_target_position[camera])

#床のパラメータ設定
plane_visual_shape_id = p.createVisualShape( #透明な床の視覚情報を設定
    shapeType=p.GEOM_PLANE,
    rgbaColor=[0., 0., 0., 1.0]  # RGBA: 赤、緑、青、アルファ（不透明度）
)
plane_collision_shape_id = p.createCollisionShape(p.GEOM_PLANE) #透明な床の衝突形状を設定
plane_id = p.createMultiBody( #透明な床を生成
    baseCollisionShapeIndex=plane_collision_shape_id,
    baseVisualShapeIndex=plane_visual_shape_id,
    basePosition=[0, 0, 0]
)

###ここまで下準備(固定)###

#迷路の盤面の作成
dimlist=[[2.5,2.5,.1],[1.7, .1, .1],[.1,.9,.1],[.9,.1,.1],[1.7,.1,.1],[.1,1.7,.1],[.9,.1,.1],
         [2.1,.1,.1],[.1,1.3,.1],[.1,1.3,.1],[.1,.5,.1],[1.7,.1,.1],[.1,.5,.1],[.5,.1,.1],
         [.9,.1,.1],[.1,.9,.1],[.9,.1,.1],[.1,.4,.1],[.5,.1,.1],
         [.3,.3,.1]
         ] #長方形のdim(z)=0.1
poslist=[[0,0,.1],[-0.8, 2.4, .3],[-2.4,1.6,.3],[-1.6,1.6,.3],[-0.8,0.8,.3],[2.4,.8,.3],[0.8,1.6,.3],
         [-0.4,0,.3],[1.6,.4,.3],[-2.4,-1.2,.3],[2.4,-2.0,.3],[0,-2.4,.3],[0.8,-2.0,.3],[1.2,-1.6,.3],
         [.8,-0.8,.3],[0,-0.8,.3],[-0.8,-1.6,.3],[-0.8,-0.4,.3],[-1.2,-0.8,.3],
         [-2.8,.4,.1]
         ] #長方形のpos(z)=0.3
rgba=[.75, .75, .75, 1.0]
basepos=[0, 0, 4]
multi_body = make_multi(dimlist,poslist,rgba,basepos)

#ゴール2の作成
dimlist2=[[1.4,.7,.1],[1.4,.1,.2],[1.4,.1,.2],[.1,.5,.2],
         ] #長方形のdim(z)=0.1
poslist2=[[0.,0.,.1],[0.,.6,.4],[0.,-0.6,.4],[-1.3,0.,.4],
         ] #長方形のpos(z)=0.3
rgba2=[.75, .75, .75, 1.0]
basepos2=[-7.5, .4, 4]
multi_body2 = make_multi(dimlist2,poslist2,rgba2,basepos2)

#球体を作成
visualShapeId = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=.2,  # 半径.2の球体
    meshScale=[1, 1, 1],  # x,y,z軸に沿ったスケール
    visualFramePosition=[0, 0, 0] #球体の視覚位置
)
collisionShapeId = p.createCollisionShape(
    shapeType=p.GEOM_SPHERE,
    radius=.2,
    meshScale=[1, 1, 1],
)
bodyId = p.createMultiBody(
    baseInertialFramePosition=[0, 0, 0],
    baseMass=0.003,
    baseCollisionShapeIndex=collisionShapeId,
    baseVisualShapeIndex=visualShapeId,
    basePosition=[-0.4, -0.4, 10], #基準位置(スタート位置)
    useMaximalCoordinates=True
)

#ゴール内衝突判定用オブジェクトを作成
visualShapeId_g = p.createVisualShape(shapeType=p.GEOM_BOX, rgbaColor=[1, 0, 0, 1], halfExtents=[.1,.5,.2])
collisionShapeId_g = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[.1,.5,.2])
goalId = p.createMultiBody(
    baseInertialFramePosition=[0, 0, 0],
    baseMass=0.,
    baseCollisionShapeIndex=collisionShapeId_g,
    baseVisualShapeIndex=visualShapeId_g,
    basePosition=[-8.6, 0.4, 4.4],
    useMaximalCoordinates=True
)

#盤面に従って一緒に動く得点オブジェクト(score)を追加、それぞれ別の物体として作成する必要がある
dimlist_s=[[[.1,.1,.1]],[[.1,.1,.1]],[[.1,.1,.1]], #dimlistとは違い三次元になっていることに注意
         ] #長方形のdim(z)=0.1
poslist_s=[[[-2.0,1.2,.3]],[[-1.2,-0.4,.3]],[[1.2,-2.0,.3]],  #poslistとは違い三次元になっていることに注意
         ] #長方形のpos(z)=0.3
rgba_s=[1., .5, .5, 1.]
basepos_s=[0, 0, 4.]
multi_body_s=[0,0,0] #それぞれのidを保存する配列
for i in range(len(dimlist_s)):
  multi_body_s[i] = make_multi(dimlist_s[i],poslist_s[i],rgba_s,basepos_s)

#それぞれの変数やフラグを初期化
p.setGravity(0, 0, -10)
stepx=0
stepy=0
anglex=0
angley=0
rad_per_step=0.002
speedup=0
goal_flag=0
score=0
score_flag=[1,1,1] #盤面に残っていたら1

#その他設定
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1) #レンダリングを一時的に無効にしてから再度有効にする
p.setRealTimeSimulation(1) #リアルタイムシミュレーションモードを有効にする
useRealTimeSimulation=1

#初期の開始時刻を設定(resetした時と揃えるため球体の位置を再度初期化)
p.resetBasePositionAndOrientation(bodyId, [-0.4, -0.4, 10], [0,0,0,1])
start = time.time()  # 現在時刻を取得


#シミュレーション開始
while p.isConnected():
    
  if (useRealTimeSimulation):  
      
    #位置と姿勢を取得 
    position, orientation = p.getBasePositionAndOrientation(multi_body) #迷路の盤面
    #print(f"Position: {position}, Orientation: {orientation}") #デバッグ用
    positionb, orientationb = p.getBasePositionAndOrientation(bodyId) #球体
    #print(f"Positionb: {positionb}, Orientationb: {orientationb}") #デバッグ用
    
    
    #キーボードイベント
    keys = p.getKeyboardEvents()
    
    #カメラ切り替え(<-カメラの拡大などができなくなる)
    if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
      camera+=1
      camera%=2
      p.resetDebugVisualizerCamera(camera_distance[camera], camera_yaw[camera], camera_pitch[camera], camera_target_position[camera])

    #盤面のリセット
    if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
      #スコア関係の初期化
      score=0
      for i in range(len(dimlist_s)):
        if(score_flag[i]==0):
          multi_body_s[i] = make_multi(dimlist_s[i],poslist_s[i],rgba_s,basepos_s) #再度物体を生成する
          score_flag[i]=1 #再度フラグを立てる
      #ボールと盤面の位置の初期化
      stepx=0
      stepy=0
      anglex=0
      angley=0
      p.resetBasePositionAndOrientation(multi_body, [0, 0, 4], [0, 0, 0, 1])
      p.resetBasePositionAndOrientation(bodyId, [-0.4, -0.4, 10], [0,0,0,1])
      p.removeAllUserDebugItems()
      start = time.time()

    #回転速度を上げる(<-最後にボールを飛ばすための機能) #X軸周りの回転については加速しないようにしている
    speedup=0
    if p.B3G_SPACE in keys and keys[p.B3G_SPACE] & p.KEY_IS_DOWN:
      speedup=6 #あげすぎるとすり抜けてしまう
    
    #X軸周りに回転
    if ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN: #手前が下がる
      #物体の新しい姿勢を計算
      stepx+=1 #+speedup
      anglex = stepx*rad_per_step  #回転角度
    if ord('u') in keys and keys[ord('u')] & p.KEY_IS_DOWN: #奥が下がる
      #物体の新しい姿勢を計算
      stepx-=1 #+speedup
      anglex = stepx*rad_per_step  #回転角度
      
    #Y軸周りに回転
    if ord('k') in keys and keys[ord('k')] & p.KEY_IS_DOWN: #右が下がる
      #物体の新しい姿勢を計算
      stepy+=1+speedup
      angley = stepy*rad_per_step  #回転角度
    if ord('h') in keys and keys[ord('h')] & p.KEY_IS_DOWN: #左が下がる
      #物体の新しい姿勢を計算
      stepy-=1+speedup
      angley = stepy*rad_per_step  #回転角度
    new_orientation = p.getQuaternionFromEuler([anglex, angley, 0])  #X,Y軸周りに回転(オイラー角からクォータニオンへの変換)
    
    
    #迷路の盤面の位置と姿勢を更新
    p.resetBasePositionAndOrientation(multi_body, position, new_orientation)
  
    #スコア用の物体の位置と姿勢を更新
    for i in range(len(dimlist_s)):
      if(score_flag[i]):
        p.resetBasePositionAndOrientation(multi_body_s[i], position, new_orientation)

    #スコア用の物体の衝突判定を行い、衝突したらスコアを加算、物体を削除する
    for i in range(len(dimlist_s)):
      if p.getContactPoints(bodyId, multi_body_s[i]):
        score_flag[i]=0
        score+=100
        p.removeBody(multi_body_s[i])
    
    #ゴールの検出
    if p.getContactPoints(bodyId, goalId):
      if(goal_flag==0):
        goal_flag=1
        end = time.time()  # 現在時刻を取得
        time_diff = end - start
        addscore=(200-time_diff)*5
        if addscore<0:
          addscore=0
        print(f"GOAL! time:{time_diff:.1f}s score:{score+addscore:.0f}")
        p.addUserDebugText(f"GOAL! time:{time_diff:.1f}s score:{score+addscore:.0f}", [-.5, -1.5, 6], [1, 0, 0], textSize=1.5,) #重いので表示は最後だけに限定 #ウィンドウ自体には表示できない

    #ボールが下に落ちたときに再度ボールを上から落下させる
    if(positionb[2]<=0.2):
      #print(positionb)
      p.resetBasePositionAndOrientation(bodyId, [-0.4, -0.4, 10], [0,0,0,1])
        
    
    sleep(.005) #0.005秒実行を止める
  else:
    p.stepSimulation() #1ステップ進める

