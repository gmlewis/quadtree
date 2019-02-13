package quadtree

import (
	"container/list"
	"math"
	"time"
)

var (
// Logger, _ = zap.NewDevelopmentConfig().Build()
)

type PhysicalObject interface {
	X() float64                // X dimension of top left corner
	Y() float64                // Y dimension of top left corner
	Width() float64            // width of the object
	Height() float64           // height of the object
	Update(time.Duration) bool // update positions of moving object
}

func Distance(one, another PhysicalObject) float64 {
	if one.X() == another.X() {
		return float64(math.Abs(float64(one.Y() - another.Y())))
	} else if one.Y() == another.Y() {
		return float64(math.Abs(float64(one.X() - another.X())))
	} else {
		return float64(math.Sqrt(
			math.Pow(
				float64(one.X()-another.X()),
				2,
			) + math.Pow(
				float64(one.Y()-another.Y()),
				2,
			),
		))
	}
}

type IntersectedObjects []PhysicalObject

// check whether current physical object intersects with another one
func Intersect(one, another PhysicalObject) bool {
	verticalOverlap := math.Abs(float64(one.Y()-another.Y())) < float64(one.Height()+another.Height())/2
	horizontalOverlap := math.Abs(float64(one.X()-another.X())) < float64(one.Width()+another.Width())/2
	if one.X() == another.X() {
		return verticalOverlap
	} else if one.Y() == another.Y() {
		return horizontalOverlap
	} else {
		return verticalOverlap && horizontalOverlap
	}
}

type Bounds struct {
	X, Y, Width, Height float64
}

// whether the physical object resides completely within bounding area of current tree, border overlaps are allowed
func (b *Bounds) Contains(obj PhysicalObject) bool {
	return obj.X() >= b.X &&
		obj.Y() >= b.Y &&
		obj.X()+obj.Width() <= b.X+b.Width &&
		obj.Y()+obj.Height() <= b.Y+b.Height
}

// Quadtree - The quadtree data structure
type Quadtree struct {
	*Bounds                    // bounds of current node
	MaxObjects    int          // Maximum objects a node can hold before splitting into 4 subnodes
	MaxLevels     int          // max number of objects in a node
	Level         int          // max level, that is, the maximum number of times a tree can be splitted up
	m_Objects     *list.List   // a list of physical objects that belongs to current node, but not children
	Nodes         [4]*Quadtree // child nodes
	m_ActiveNodes byte
	m_curLife     int
	m_maxLifespan int
	m_parent      *Quadtree
}

// intersection infomation between two physical objects
type IntersectionRecord struct {
	One     PhysicalObject
	Another PhysicalObject
}

// BuildTree determines whether to subdevide according to number of m_Objects, and the current level.
// Upon subdeviding, it only create&replace neccessary sub trees
func (qt *Quadtree) Build() {
	if qt.m_Objects.Len() <= qt.MaxObjects || qt.Level >= qt.MaxLevels {
		return
	}

	subBounds := [4]*Bounds{
		// top left
		&Bounds{qt.X, qt.Y, qt.Width / 2, qt.Height / 2},
		// top right
		&Bounds{qt.X + qt.Width/2, qt.Y, qt.Width / 2, qt.Height / 2},
		// bottom left
		&Bounds{qt.X, qt.Y + qt.Height/2, qt.Width / 2, qt.Height / 2},
		// bottom right
		&Bounds{qt.X + qt.Width/2, qt.Y + qt.Height/2, qt.Width / 2, qt.Height / 2},
	}

	var subtreeObjects [4][]PhysicalObject

	horizontalMidpoint := qt.X + (qt.Width / 2)
	verticalMidpoint := qt.Y + (qt.Height / 2)
	var delist []*list.Element

	for ele := qt.m_Objects.Front(); ele != nil; ele = ele.Next() {
		obj := ele.Value.(PhysicalObject)

		topPart := (obj.Y() >= qt.Y) && (obj.Y()+obj.Height() <= verticalMidpoint)
		bottomPart := (obj.Y() >= verticalMidpoint) && (obj.Y()+obj.Height() <= qt.Height)
		leftPart := (obj.X() >= qt.X) && (obj.X()+obj.Width() <= horizontalMidpoint)
		rightPart := (obj.X() >= horizontalMidpoint) && (obj.X()+obj.Width() <= qt.Width)

		index := -1
		// obj can completely fit within the left quadrants
		if topPart {
			if leftPart {
				index = 0
			} else if rightPart {
				index = 1
			}
		} else if bottomPart {
			if leftPart {
				index = 2
			} else if rightPart {
				index = 3
			}
		}
		// Logger.Info("object index", zap.Int("index", index))

		if index != -1 {
			delist = append(delist, ele)
			subtreeObjects[index] = append(subtreeObjects[index], obj)
		}
	}

	for _, ele := range delist {
		qt.m_Objects.Remove(ele)
	}

	for i, objects := range subtreeObjects {
		if len(objects) > 0 {
			qt.Nodes[i] = qt.createSubtree(subBounds[i], objects...)
			qt.Nodes[i].Build()
			qt.m_ActiveNodes |= 1 << uint(i)
		}
	}
}

// UpdateTree rebuild the tree using the specified objects
func (qt *Quadtree) UpdateTree(objects *list.List) {
	qt.m_ActiveNodes = 0
	qt.Nodes = [4]*Quadtree{}
	qt.m_Objects = objects
	qt.Build()
}

// Update physical objects and maintain states of the tree
func (qt *Quadtree) Update(delta time.Duration) {
	if qt.m_Objects.Len() == 0 {
		// 当物体一个Node中的物体移动出去之后，如果没有其他物体进入，该Node还会存留m_maxLifespan个生命周期
		if qt.m_ActiveNodes == 0 {
			if qt.m_curLife == -1 {
				qt.m_curLife = qt.m_maxLifespan
				qt.m_curLife -= 1
			} else if qt.m_curLife > 0 {
				qt.m_curLife -= 1
			}
		}
	} else {
		// 只要节点直接有物体或者字节点中有物体，所有生命倒计时停止
		if qt.m_curLife != -1 {
			if qt.m_maxLifespan <= 64 {
				qt.m_maxLifespan *= 2
			}
			qt.m_curLife = -1
		}
	}

	// update physical objects
	var movedObjects []*list.Element
	for ele := qt.m_Objects.Front(); ele != nil; ele = ele.Next() {
		obj := ele.Value.(PhysicalObject)
		// Logger.Info("updating object previously located at", zap.Float64("X", obj.X()), zap.Float64("Y", obj.Y()))
		if obj.Update(delta) {
			// Logger.Info("object moved to", zap.Float64("X", obj.X()), zap.Float64("Y", obj.Y()))
			movedObjects = append(movedObjects, ele)
		}
	}

	// update child nodes
	flags := qt.m_ActiveNodes
	index := 0
	for flags > 0 {
		if flags&1 == 1 {
			qt.Nodes[index].Update(delta)
		}
		flags >>= 1
		index += 1
	}

	// move updated physical objects
	for _, ele := range movedObjects {
		container := qt
		obj := ele.Value.(PhysicalObject)
		for !container.Contains(obj) {
			if container.m_parent != nil {
				container = container.m_parent
			} else {
				break
			}
		}
		qt.m_Objects.Remove(ele)
		/*
			Logger.Info(
				"object about moved to container",
				zap.Float64("object X", obj.X()),
				zap.Float64("object Y", obj.Y()),
				zap.Float64("container X", container.X),
				zap.Float64("container Y", container.Y),
				zap.Float64("container width", container.Width),
				zap.Float64("container height", container.Height),
			)
		*/
		container.Insert(obj)
	}

	// prune out dead subtree
	flags = qt.m_ActiveNodes
	index = 0
	for flags > 0 {
		if flags&1 == 1 && qt.Nodes[index].m_curLife == 0 {
			qt.Nodes[index] = nil
			qt.m_ActiveNodes = qt.m_ActiveNodes &^ (1 << uint(index))
		}
		flags >>= 1
		index += 1
	}
}

// Insert - Insert the object into the node. If the node exceeds the capacity,
// it will split and add all objects to their corresponding subnodes.
// Caller needs to make sure the physical object to be inserted is completely contained withing this node
func (qt *Quadtree) Insert(physical PhysicalObject) {
	/*
		Logger.Info(
			"inserting physical object",
			zap.Float64("object X", physical.X()),
			zap.Float64("object Y", physical.Y()),
			zap.Float64("object Width", physical.Width()),
			zap.Float64("object Height", physical.Height()),
			zap.Float64("tree X", qt.X),
			zap.Float64("tree Y", qt.Y),
			zap.Float64("tree Width", qt.Width),
			zap.Float64("tree Height", qt.Height),
		)
	*/
	if qt.m_ActiveNodes == 0 {
		qt.m_Objects.PushBack(physical)
		// simply add to list if no subtree and there is no need to create one
		if qt.m_Objects.Len() < qt.MaxObjects || qt.Level == qt.MaxLevels {
			// Logger.Info("simply add to list if no subtree and there is no need to create one")
		} else {
			// rebuild the tree
			// Logger.Info("rebuild the tree, since new objects entering the region")
			qt.Build()
		}
		return
	}

	horizontalMidpoint := qt.X + (qt.Width / 2)
	verticalMidpoint := qt.Y + (qt.Height / 2)

	topPart := (physical.Y() >= qt.Y) && (physical.Y()+physical.Height() <= verticalMidpoint)
	bottomPart := (physical.Y() >= verticalMidpoint) && (physical.Y()+physical.Height() <= qt.Height)
	leftPart := (physical.X() >= qt.X) && (physical.X()+physical.Width() <= horizontalMidpoint)
	rightPart := (physical.X() >= horizontalMidpoint) && (physical.X()+physical.Width() <= qt.Width)

	index := -1
	//pRect can completely fit within the left quadrants
	if topPart {
		if leftPart {
			index = 0
		} else if rightPart {
			index = 1
		}
	} else if bottomPart {
		if leftPart {
			index = 2
		} else if rightPart {
			index = 3
		}
	}

	if index == -1 {
		qt.m_Objects.PushBack(physical)
	} else {
		if qt.m_ActiveNodes&(1<<uint(index)) == 0 {
			var bounds *Bounds
			switch index {
			case 0:
				// top left
				bounds = &Bounds{qt.X, qt.Y, qt.Width / 2, qt.Height / 2}
			case 1:
				// top right
				bounds = &Bounds{qt.X + qt.Width/2, qt.Y, qt.Width / 2, qt.Height / 2}
			case 2:
				// bottom left
				bounds = &Bounds{qt.X, qt.Y + qt.Height/2, qt.Width / 2, qt.Height / 2}
			case 3:
				// bottom right
				bounds = &Bounds{qt.X + qt.Width/2, qt.Y + qt.Height/2, qt.Width / 2, qt.Height / 2}
			}
			// create subtree if not exists
			qt.Nodes[index] = qt.createSubtree(bounds)
			qt.m_ActiveNodes |= 1 << uint(index)
			// Logger.Info("create subtree", zap.Int("index", index), zap.Any("bounds", bounds))
		}
		// insert into subtree
		// Logger.Info("insert into subtree", zap.Int("subtree index", index))
		qt.Nodes[index].Insert(physical)
	}
}

// Remove a physical object from the quadtree
func (qt *Quadtree) Remove(target PhysicalObject) bool {
	for ele := qt.m_Objects.Front(); ele != nil; ele = ele.Next() {
		one := ele.Value.(PhysicalObject)
		if one == target {
			qt.m_Objects.Remove(ele)
			return true
		}
	}

	flags := qt.m_ActiveNodes
	index := 0
	for flags > 0 {
		if flags&1 == 1 {
			if removed := qt.Nodes[index].Remove(target); removed {
				return true
			}
		}
		flags >>= 1
		index += 1
	}
	return false
}

// 广度优先遍历
func (qt *Quadtree) Walk(walker func(PhysicalObject)) {
	for ele := qt.m_Objects.Front(); ele != nil; ele = ele.Next() {
		walker(ele.Value.(PhysicalObject))
	}
	flags := qt.m_ActiveNodes
	index := 0
	for flags > 0 {
		if flags&1 == 1 {
			qt.Nodes[index].Walk(walker)
		}
		flags >>= 1
		index += 1
	}
}

// FindObject returns the Quadtree that directly contains the physical object
// TODO: 根据target的位置区间加快搜索
func (qt *Quadtree) FindObject(target PhysicalObject) *Quadtree {
	for ele := qt.m_Objects.Front(); ele != nil; ele = ele.Next() {
		one := ele.Value.(PhysicalObject)
		if one == target {
			return qt
		}
	}

	flags := qt.m_ActiveNodes
	index := 0
	for flags > 0 {
		if flags&1 == 1 {
			if sub := qt.Nodes[index].FindObject(target); sub != nil {
				return sub
			}
		}
		flags >>= 1
		index += 1
	}
	return nil
}

//
func (qt *Quadtree) GetIntersectedObjectsRaw(target PhysicalObject, objects []PhysicalObject) IntersectedObjects {
	for ele := qt.m_Objects.Front(); ele != nil; ele = ele.Next() {
		obj := ele.Value.(PhysicalObject)
		if obj == target {
			continue
		}
		if Intersect(target, obj) {
			objects = append(objects, obj)
		}
	}

	flags := qt.m_ActiveNodes
	index := 0
	for flags > 0 {
		if flags&1 == 1 {
			objects = qt.Nodes[index].GetIntersectedObjectsRaw(target, objects)
		}
		flags >>= 1
		index += 1
	}
	return objects
}

func (qt *Quadtree) GetIntersectedObjects(target PhysicalObject) IntersectedObjects {
	sub := qt.FindObject(target)
	if sub == nil {
		return nil
	}

	var objects []PhysicalObject
	// find intersected objects in parent trees
	parent := sub.m_parent
	for parent != nil {
		for ele := parent.m_Objects.Front(); ele != nil; ele = ele.Next() {
			obj := ele.Value.(PhysicalObject)
			if obj == target {
				continue
			}
			if Intersect(target, obj) {
				objects = append(objects, obj)
			}
		}
		parent = parent.m_parent
	}

	// find intersected objects in current tree and its children
	return sub.GetIntersectedObjectsRaw(target, objects)
}

// get a list of intersection records within this quadtree
func (qt *Quadtree) GetIntersection(intersections *list.List, potentialObjects *list.List) *list.List {
	if intersections == nil {
		intersections = &list.List{}
	}
	if potentialObjects == nil {
		potentialObjects = &list.List{}
	}
	for ele := qt.m_Objects.Front(); ele != nil; ele = ele.Next() {
		one := ele.Value.(PhysicalObject)
		// check intersections with each physical object of parent nodes, or previous objects in current node
		for eleParent := potentialObjects.Front(); eleParent != nil; eleParent = eleParent.Next() {
			objParent := eleParent.Value.(PhysicalObject)
			if Intersect(objParent, one) {
				intersections.PushBack(&IntersectionRecord{
					One:     objParent,
					Another: one,
				})
			}
		}
		potentialObjects.PushBack(one)
	}

	flags := qt.m_ActiveNodes
	index := 0
	for flags > 0 {
		if flags&1 == 1 {
			qt.Nodes[index].GetIntersection(intersections, potentialObjects)
		}
		flags >>= 1
		index += 1
	}
	return intersections
}

// initialize a quadtree
func CreateQuadtree(bounds *Bounds,
	maxObjectsBeforeSplit,
	maxLevelsToSplit int,
	physicalObjects ...PhysicalObject) *Quadtree {

	objects := &list.List{}
	for _, obj := range physicalObjects {
		objects.PushBack(obj)
	}
	return &Quadtree{
		Bounds:        bounds,
		MaxObjects:    maxObjectsBeforeSplit,
		MaxLevels:     maxLevelsToSplit,
		m_Objects:     objects,
		m_curLife:     -1,
		m_maxLifespan: 64,
	}
}

func (qt *Quadtree) createSubtree(bounds *Bounds, physicals ...PhysicalObject) *Quadtree {
	subtree := CreateQuadtree(bounds, qt.MaxObjects, qt.MaxLevels, physicals...)
	subtree.Level = qt.Level + 1
	subtree.m_parent = qt
	return subtree
}
