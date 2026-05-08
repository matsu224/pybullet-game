import pybullet as p
import pybullet_data
import time
import math
import random
from collections import deque

#迷路をランダム生成するバージョンの完成形

#BFSで迷路の最も遠い点を求める
def bfs(maze, start):
    rows, cols = len(maze), len(maze[0])
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 上下左右の移動
    visited = [[False] * cols for _ in range(rows)]
    queue = deque([(start, 0)])
    visited[start[0]][start[1]] = True
    goal = start
    max_distance = 0

    while queue:
        (x, y), dist = queue.popleft()
        if dist > max_distance:
            max_distance = dist
            goal = (x, y)
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols and not visited[nx][ny] and maze[nx][ny] == 0:
                visited[nx][ny] = True
                queue.append(((nx, ny), dist + 1))
    
    return goal, max_distance

#迷路のリストをランダム生成する
def make_maze(MAZE_W = 11,MAZE_H = 11):
    #リストの宣言と初期化
    maze = []
    for y in range(MAZE_H):
        maze.append([0] * MAZE_W)
    
    #柱から伸ばす壁のに利用する値を定義
    #[上, 右, 下, 左]
    XP = [0, 1, 0, -1]
    YP = [-1, 0, 1, 0]

    #迷路を囲う壁を作る
    for x in range(MAZE_W):
        maze[0][x] = 1
        maze[MAZE_H - 1][x] = 1
    for y in range(1, MAZE_H - 1):
        maze[y][0] = 1
        maze[y][MAZE_W - 1] = 1
    
    #中を何もない状態にする
    for y in range(1, MAZE_H - 1):
        for x in range(1, MAZE_W - 1):
            maze[y][x] = 0
    
    #柱を作る
    for y in range(2, MAZE_H - 2, 2):           #range()は第三引数を2を指定し、ステップ機能で1マス飛ばししている
        for x in range(2, MAZE_W - 2, 2):
            maze[y][x] = 1
    
    #各柱から壁を伸ばす
    for y in range(2, MAZE_H - 2, 2):
        for x in range(2, MAZE_W - 2, 2):
            while True:
                d = random.randint(0, 3)            #変数dに柱から伸ばす方向を0~3で指定
                if x > 2:                           #2列目以降なら0~2（左を示す3を含めない）で左に伸ばさない
                    d = random.randint(0, 2)
                
                if maze[y + YP[d]][x + XP[d]] == 1: # dの値が既に壁が作られた場所であればやり直し
                    continue

                #柱から伸ばす壁を示す値（変数d）を、定数YP、XPの添字に使い壁を伸ばすマス目を指定
                #そのマス目を表すmaze[]に壁有りを示す1を代入
                maze[y + YP[d]][x + XP[d]] = 1
                break
    
    return maze

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
        linkOrientations=lo, #親リンクからの子リンクの方向・回転)
        linkInertialFramePositions=poslist, #???
        linkInertialFrameOrientations=lifo, #???
        linkParentIndices=lpi, #???
        linkJointTypes=ljt, #ジョイントの種類
        linkJointAxis=lja, #ジョイントの軸 #???
    )
    
    return multi_body

def make_maze_and_body(maze_w_in=9,maze_h_in=9,block_half_wh_in=0.1,path_half_wh_in=0.3):
    
    global maze_w,maze_h,block_half_wh,path_half_wh,half_w,half_h
    global multi_body,sub_multi_body,goalId #ballIdは関数内で定義しないようにする
    global basepos,rad_per_step #ball_basepos,ball_radiusも同様に関数内で定義しない
    
    #各種変数設定、初期化設定
    maze_w = maze_w_in #x
    maze_h = maze_h_in #y
    #壁と道の大きさ(半分)
    block_half_wh=block_half_wh_in
    path_half_wh=path_half_wh_in
    #盤面全体の大きさの半分を求める
    half_w=block_half_wh*(maze_w//2+1)+path_half_wh*(maze_w//2)
    half_h=block_half_wh*(maze_h//2+1)+path_half_wh*(maze_h//2)
    #基準位置
    basepos=[0,0,((half_w**2+half_h**2)**0.5)*1.05]
    #rgba設定
    body_rgba=[.75, .75, .75, 1.0]
    sub_body_rgba=[.90, .90, .90, 1.0]
    goal_rgba=[1,0,0,1]
    #ステップあたりの回転角度
    rad_per_step=math.acos(1-1/(2*(half_w**2+half_h**2)))/100
    #カメラの初期化設定
    camera_distance = half_h*1.5
    camera_yaw = 0 #水平方向の回転角度
    camera_pitch = -60 #垂直方向の回転角度
    camera_target_position = basepos #カメラのターゲットとなる位置
    p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

    #迷路の作成
    maze=make_maze(maze_h,maze_w) #mazeのi行->x、j列->yとなるように(maze_h,maze_w)の順番にしていることに注意
    #ゴールの位置の取得
    goalpos,distance=bfs(maze, [1,1])
    goalpos=[[-half_w+block_half_wh+(block_half_wh+path_half_wh)*goalpos[0],
            -half_h+block_half_wh+(block_half_wh+path_half_wh)*goalpos[1],
            block_half_wh*3]] #make_multi用に二次元配列とする
    #print(goalpos)
        
    #盤面の作成
    #リストの宣言と初期化
    dimlist=[[half_w,half_h,block_half_wh]]
    poslist=[[0,0,block_half_wh]]

    #マルチボディの基礎を作成（位置や角度の代表とするため、一番下の平面）
    multi_body = make_multi(dimlist,poslist,body_rgba,basepos)

    #ブロックを追加
    sub_multi_body=[]
    for i in range(len(maze)):
        dimlist=[]
        poslist=[]
        for j in range(len(maze[0])):
            if(maze[i][j]==1):
                xdim=block_half_wh if i%2==0 else path_half_wh
                ydim=block_half_wh if j%2==0 else path_half_wh
                xpos=-half_w+block_half_wh+(block_half_wh+path_half_wh)*i
                ypos=-half_h+block_half_wh+(block_half_wh+path_half_wh)*j
                dimlist.append([xdim,ydim,block_half_wh*2])
                poslist.append([xpos,ypos,block_half_wh*4])
        #iのループごとにマルチボディを分割
        sub_multi_body.append(make_multi(dimlist,poslist,sub_body_rgba,basepos))

    #ゴール用衝突判定用オブジェクトを作成
    goalId=make_multi([[path_half_wh,path_half_wh,block_half_wh]],goalpos,goal_rgba,basepos)
    
    
def update_ball():
    
    global ball_basepos,ball_radius
    
    #ボールの初期設定、または設定の更新
    ball_basepos=[-half_w+block_half_wh*2+path_half_wh, -half_h+block_half_wh*2+path_half_wh, ((half_w**2+half_h**2)**0.5)*1.05*2]
    ball_radius=block_half_wh*2 #球体の半径

    
def make_ball():
    
    global ballId
    
    #球体を作成
    visualShapeId = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=ball_radius,
        meshScale=[1, 1, 1],  # x,y,z軸に沿ったスケール
        visualFramePosition=[0, 0, 0] #球体の視覚位置
    )
    collisionShapeId = p.createCollisionShape(
        shapeType=p.GEOM_SPHERE,
        radius=ball_radius,
        meshScale=[1, 1, 1],
    )
    ballId = p.createMultiBody(
        baseInertialFramePosition=[0, 0, 0],
        baseMass=0.003,
        baseCollisionShapeIndex=collisionShapeId,
        baseVisualShapeIndex=visualShapeId,
        basePosition=ball_basepos, #基準位置(スタート位置)
        useMaximalCoordinates=True
    )
    

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
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0) #キーボードショートカットを無効にする

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

#サイズに関する変数の初期値を設定
maze_w_in=9
maze_h_in=9
block_half_wh_in=0.1
path_half_wh_in=0.3

#シミュレーション時に必要な（サイズに影響されない）それぞれの変数やフラグを初期化
stepx=0
stepy=0
anglex=0
angley=0
speedup=0
goal_flag=0

#その他設定
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1) #リアルタイムシミュレーションモードを有効にする

#初期の迷路、ボールの作成、開始時刻を設定
make_maze_and_body(maze_w_in,maze_h_in,block_half_wh_in,path_half_wh_in)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1) #レンダリングを一時的に無効にしてから再度有効にする
update_ball()
make_ball()
start = time.time()  # 現在時刻を取得


#シミュレーション開始
while p.isConnected():  
      
    #位置と姿勢を取得 
    position, orientation = p.getBasePositionAndOrientation(multi_body) #迷路の盤面
    #print(f"Position: {position}, Orientation: {orientation}") #デバッグ用
    positionb, orientationb = p.getBasePositionAndOrientation(ballId) #球体
    #print(f"Positionb: {positionb}, Orientationb: {orientationb}") #デバッグ用
    
    
    #キーボードイベント
    keys = p.getKeyboardEvents()
    
    #盤面のリセット
    if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
      #ボールと盤面の位置の初期化
      goal_flag=0
      stepx=0
      stepy=0
      anglex=0
      angley=0
      p.resetBasePositionAndOrientation(multi_body, basepos, [0, 0, 0, 1])
      p.resetBasePositionAndOrientation(ballId, ball_basepos, [0,0,0,1])
      #p.removeAllUserDebugItems()
      start = time.time()
      
    #回転速度を上げる
    speedup=0
    if p.B3G_SPACE in keys and keys[p.B3G_SPACE] & p.KEY_IS_DOWN:
      speedup=2 #あげすぎるとすり抜けてしまう
    
    #X軸周りに回転
    if ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN: #手前が下がる
      #物体の新しい姿勢を計算
      stepx+=1+speedup
      anglex = stepx*rad_per_step  #回転角度
    if ord('u') in keys and keys[ord('u')] & p.KEY_IS_DOWN: #奥が下がる
      #物体の新しい姿勢を計算
      stepx-=1+speedup
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
    
    
    #盤面の基準平面の位置と姿勢を更新
    p.resetBasePositionAndOrientation(multi_body, position, new_orientation)
    #分割された盤面の位置と姿勢を更新（iが大きすぎるとこの処理が重くなる）
    for i in range(len(sub_multi_body)):
        p.resetBasePositionAndOrientation(sub_multi_body[i], position, new_orientation)
    #ゴールの位置と姿勢を更新
    p.resetBasePositionAndOrientation(goalId, position, new_orientation)
    
    #ゴールの検出
    if p.getContactPoints(ballId, goalId):
        if(goal_flag==0):
            goal_flag=1
            end = time.time()  # 現在時刻を取得
            time_diff = end - start
            text=f"GOAL! time:{time_diff:.1f}s"
            text_len=len(text)
            print(text)
            #p.addUserDebugText(text, [-0.1*text_len/2+basepos[0], basepos[1], basepos[2]*1.5], [1, 0, 0], textSize=3.0,) #重いので表示は最後だけに限定 #ウィンドウ自体には表示できない

    #ゴール後に次の盤面に移行する
    if ord('n') in keys and keys[ord('n')] & p.KEY_WAS_TRIGGERED:
        #if(goal_flag==1):
            #ボールと盤面の位置の初期化
            goal_flag=0
            stepx=0
            stepy=0
            anglex=0
            angley=0
            #p.removeAllUserDebugItems()
            if(maze_w_in<50): #上限の設定
                maze_w_in+=2
            if(maze_h_in<50): #上限の設定
                maze_h_in+=2
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
            p.removeBody(multi_body)
            for i in range(len(sub_multi_body)):
                p.removeBody(sub_multi_body[i])
            #p.removeBody(ballId) #なぜかこれだけ消えない
            p.removeBody(goalId)
            make_maze_and_body(maze_w_in,maze_h_in,block_half_wh_in,path_half_wh_in)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            update_ball()
            p.resetBasePositionAndOrientation(ballId, ball_basepos, [0,0,0,1]) #ballは使い回す
            start = time.time()
            

    #ボールが下に落ちたときに再度ボールを上から落下させる
    if(positionb[2]<=ball_radius):
      p.resetBasePositionAndOrientation(ballId, ball_basepos, [0,0,0,1])
        

    time.sleep(.005)  # Time in seconds. #0.005秒実行を止める

