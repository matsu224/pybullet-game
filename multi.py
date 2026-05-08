import pybullet as p

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