package quadtree

import (
	"bytes"
	"fmt"
	"strings"
	"testing"
	"time"
)

type TestPhysicalObject struct {
	X, Y, Width, Height float32
}

func (po *TestPhysicalObject) GetX() float32 {
	return po.X
}

func (po *TestPhysicalObject) GetY() float32 {
	return po.Y
}

func (po *TestPhysicalObject) GetWidth() float32 {
	return po.Width
}

func (po *TestPhysicalObject) GetHeight() float32 {
	return po.Height
}

func (po *TestPhysicalObject) Update(delta time.Duration) bool {
	return true
}

// TestSetup defined data to create a Quadtree
type TestSetup struct {
	X, Y, Width, Height   float32
	MaxObjects, MaxLevels int
	PhysicalObjects       []float32 // groups of (X, Y, Width, Height)
}

// QuadtreeState defines the expected state of a Quadtree
type QuadtreeState struct {
	PhysicalObjects []float32         // groupds of (X, Y, Width, Height), representing objects in the root node
	SubTrees        [4]*QuadtreeState // nil element to identify that no such subtree should be created
}

func (qs *QuadtreeState) String(indent int) string {
	var buf bytes.Buffer
	indentString := strings.Repeat("\t", indent)

	for i := 0; i < len(qs.PhysicalObjects); i += 4 {
		buf.WriteString(indentString)
		buf.WriteString(
			fmt.Sprintf(
				"%-10.2f%-10.2f%-10.2f%-10.2f\n",
				qs.PhysicalObjects[i],
				qs.PhysicalObjects[i+1],
				qs.PhysicalObjects[i+2],
				qs.PhysicalObjects[i+3],
			),
		)
	}
	for i, subTree := range qs.SubTrees {
		if subTree == nil {
			continue
		}
		buf.WriteString(indentString)
		buf.WriteString(fmt.Sprintf("%d:\n", i))
		buf.WriteString(subTree.String(indent + 1))
	}
	return buf.String()
}

func (realState *QuadtreeState) Check(state *QuadtreeState) bool {
	if len(state.PhysicalObjects) != len(realState.PhysicalObjects) {
		return false
	}

	usedIndex := map[int]bool{}
	for i := 0; i < len(state.PhysicalObjects); i = i + 4 {
		found := false
		for k := 0; k < len(realState.PhysicalObjects); k += 4 {
			if !usedIndex[k] &&
				realState.PhysicalObjects[k] == state.PhysicalObjects[i] &&
				realState.PhysicalObjects[k+1] == state.PhysicalObjects[i+1] &&
				realState.PhysicalObjects[k+2] == state.PhysicalObjects[i+2] &&
				realState.PhysicalObjects[k+3] == state.PhysicalObjects[i+3] {

				found = true
				usedIndex[k] = true
				break
			}
		}
		if !found {
			return false
		}
	}

	for i, subTreeState := range state.SubTrees {
		if subTreeState == nil {
			// subtree created when should not
			if realState.SubTrees[i] != nil {
				return false
			}
		} else {
			// subtree not created when shoud
			if realState.SubTrees[i] == nil {
				return false
			} else {
				// evaluates state of subtree
				return realState.SubTrees[i].Check(subTreeState)
			}
		}
	}

	return true
}

func (qt *Quadtree) DumpState() *QuadtreeState {
	state := &QuadtreeState{}
	for ele := qt.m_Objects.Front(); ele != nil; ele = ele.Next() {
		obj := ele.Value.(PhysicalObject)
		state.PhysicalObjects = append(state.PhysicalObjects, obj.GetX(), obj.GetY(), obj.GetWidth(), obj.GetHeight())
	}

	flags := qt.m_ActiveNodes
	index := 0
	for flags > 0 {
		if flags&1 == 1 && qt.Nodes[index] != nil {
			state.SubTrees[index] = qt.Nodes[index].DumpState()
		}
		flags >>= 1
		index += 1
	}
	return state
}

// list of pair of PhysicalObject representing intersected objects
type QuadtreeIntersections []PhysicalObject

func SameAs(obj PhysicalObject, another PhysicalObject) bool {
	return obj.GetX() == another.GetX() &&
		obj.GetY() == another.GetY() &&
		obj.GetWidth() == another.GetWidth() &&
		obj.GetHeight() == another.GetHeight()
}

func (qt *Quadtree) DumpIntersections() QuadtreeIntersections {
	intersectionList := qt.GetIntersection(nil, nil)
	var intersections []PhysicalObject
	for ele := intersectionList.Front(); ele != nil; ele = ele.Next() {
		record := ele.Value.(*IntersectionRecord)
		intersections = append(intersections, record.One, record.Another)
	}
	return intersections
}

func (actual QuadtreeIntersections) Check(expected QuadtreeIntersections) bool {
	if len(actual) != len(expected) {
		return false
	}
	usedIndex := make(map[int]bool)
	for i := 0; i < len(actual); i += 2 {
		found := false
		for k := 0; k < len(expected); k += 2 {
			if !usedIndex[k] &&
				(SameAs(actual[i], expected[k]) && SameAs(actual[i+1], expected[k+1])) ||
				(SameAs(actual[i], expected[k+1]) && SameAs(actual[i+1], expected[k])) {

				found = true
				usedIndex[k] = true
				break
			}
		}
		if !found {
			return false
		}
	}
	return true
}

func (inter QuadtreeIntersections) String() string {
	if len(inter) == 0 {
		return "None"
	}

	var buf bytes.Buffer
	for i := 0; i < len(inter); i += 2 {
		one := inter[i]
		another := inter[i+1]
		buf.WriteString(
			fmt.Sprintf(
				"(%-10.2f%-10.2f%-10.2f%-10.2f) (%-10.2f%-10.2f%-10.2f%-10.2f)\n",
				one.GetX(), one.GetY(), one.GetWidth(), one.GetHeight(),
				another.GetX(), another.GetY(), another.GetWidth(), another.GetHeight(),
			),
		)
	}
	return buf.String()
}

func (inter IntersectedObjects) SameAs(another IntersectedObjects) bool {
	if len(inter) != len(another) {
		return false
	}

	usedIndex := make(map[int]bool)
	for _, one := range inter {
		found := false
		for k, two := range another {
			if !usedIndex[k] && one.GetX() == two.GetX() && one.GetY() == two.GetY() && one.GetWidth() == two.GetWidth() && one.GetHeight() == two.GetHeight() {
				found = true
				usedIndex[k] = true
				break
			}
		}
		if !found {
			return false
		}
	}
	return true
}

func (inter IntersectedObjects) String() string {
	var buf bytes.Buffer
	for _, obj := range inter {
		buf.WriteString(
			fmt.Sprintf(
				"%-10.2f%-10.2f%-10.2f%-10.2f",
				obj.GetX(), obj.GetY(), obj.GetWidth(), obj.GetHeight(),
			),
		)
	}
	return buf.String()
}

type TestExpectation struct {
	*QuadtreeState
	QuadtreeIntersections
}

type OperationFunc func(*Quadtree, []PhysicalObject) []interface{}
type ExpectationFunc func(*testing.T, int, []interface{})

// TestOperation defined an operation bo be invoked on a Quadtree, and the expectation
type TestOperation struct {
	Operation   OperationFunc
	Expectation []ExpectationFunc
}

type TestCase struct {
	Setup      *TestSetup
	Operations []*TestOperation
}

/* ++++++++++ START pre-defined TestOperation*/
func OP_Build(qt *Quadtree, _ []PhysicalObject) []interface{} {
	qt.Build()
	return []interface{}{qt}
}

func OP_Insert(parts ...float32) OperationFunc {
	return func(qt *Quadtree, _ []PhysicalObject) []interface{} {
		for i := 0; i < len(parts); i += 4 {
			qt.Insert(&TestPhysicalObject{
				X:      parts[i],
				Y:      parts[i+1],
				Width:  parts[i+2],
				Height: parts[i+3],
			})
		}
		return []interface{}{qt}
	}
}

func OP_Remove(index int) OperationFunc {
	return func(qt *Quadtree, objects []PhysicalObject) []interface{} {
		qt.Remove(objects[index])
		return []interface{}{qt}
	}
}

func OP_UpdateObject(index int, x, y float32, updateTimes int) OperationFunc {
	return func(qt *Quadtree, objects []PhysicalObject) []interface{} {
		obj := objects[index].(*TestPhysicalObject)
		obj.X = x
		obj.Y = y

		for i := 0; i < updateTimes; i += 1 {
			qt.Update(0 * time.Second)
		}
		return []interface{}{qt}
	}
}

func OP_GetIntersectedObjects(index int) OperationFunc {
	return func(qt *Quadtree, objects []PhysicalObject) []interface{} {
		return []interface{}{qt, qt.GetIntersectedObjects(objects[index])}
	}
}

func OP_FindObject(index int) OperationFunc {
	return func(qt *Quadtree, objects []PhysicalObject) []interface{} {
		tree := qt.FindObject(objects[index])
		return []interface{}{tree}
	}
}

func EX_CheckIntersectedObjects(expectedInter IntersectedObjects) ExpectationFunc {
	return func(t *testing.T, testIndex int, params []interface{}) {
		qt := params[0].(*Quadtree)
		realInter := params[1].(IntersectedObjects)

		if !realInter.SameAs(expectedInter) {
			t.Errorf("Object in Quadtree (%d) expectes intersection:\n%s\nBut has intersection:\n%s\nIts state:\n%s",
				testIndex,
				expectedInter.String(),
				realInter.String(),
				qt.DumpState().String(0),
			)
		}
	}
}

func EX_CheckStateAndIntersections(expectation *TestExpectation) ExpectationFunc {
	return func(t *testing.T, testIndex int, params []interface{}) {
		qt := params[0].(*Quadtree)

		realState := qt.DumpState()
		expectedState := expectation.QuadtreeState
		if !realState.Check(expectedState) {
			t.Errorf(
				"\nQuadtree (%d) expectes to be in state:\n%s\nBut in state:\n%s",
				testIndex,
				expectedState.String(0),
				realState.String(0),
			)
		}
		realIntersections := qt.DumpIntersections()
		expectedIntersections := expectation.QuadtreeIntersections
		if !realIntersections.Check(expectedIntersections) {
			t.Errorf(
				"\nQuadtree (%d) expectes to have intersections:\n%s\nBut has:\n%s\nIts state:\n%s",
				testIndex,
				expectedIntersections.String(),
				realIntersections.String(),
				realState.String(0),
			)
		}
	}
}

func EX_CheckState(expectedState *QuadtreeState) ExpectationFunc {
	return func(t *testing.T, testIndex int, params []interface{}) {
		qt := params[0].(*Quadtree)

		realState := qt.DumpState()
		if !realState.Check(expectedState) {
			t.Errorf(
				"\nQuadtree (%d) expectes to be in state:\n%s\nBut in state:\n%s",
				testIndex,
				expectedState.String(0),
				realState.String(0),
			)
		}
	}
}

/* ========== END pre-defined TestOperation*/
var (
	testCases = []*TestCase{
		&TestCase{ // Remove, 叶子节点
			Setup: &TestSetup{
				0, 0, 2, 2,
				1, 10,
				[]float32{
					0.5, 0.5, 1, 1,
					0, 0, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{0.5, 0.5, 1, 1},
							[4]*QuadtreeState{
								&QuadtreeState{
									[]float32{0, 0, 1, 1},
									[4]*QuadtreeState{},
								},
							},
						},
					)},
				},
				&TestOperation{
					Operation: OP_Remove(1),
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{0.5, 0.5, 1, 1},
							[4]*QuadtreeState{
								&QuadtreeState{},
							},
						},
					)},
				},
			},
		},
		&TestCase{ // Remove, 非叶子节点
			Setup: &TestSetup{
				0, 0, 2, 2,
				1, 10,
				[]float32{
					0.5, 0.5, 1, 1,
					0, 0, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{0.5, 0.5, 1, 1},
							[4]*QuadtreeState{
								&QuadtreeState{
									[]float32{0, 0, 1, 1},
									[4]*QuadtreeState{},
								},
							},
						},
					)},
				},
				&TestOperation{
					Operation: OP_Remove(0),
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{},
							[4]*QuadtreeState{
								&QuadtreeState{
									[]float32{0, 0, 1, 1},
									[4]*QuadtreeState{},
								},
							},
						},
					)},
				},
			},
		},
		&TestCase{ // GetIntersectedObjects, 跨级，从子节点中找Intersection
			Setup: &TestSetup{
				0, 0, 4, 4,
				1, 10,
				[]float32{
					1.5, 1, 1, 1,
					0, 0, 1, 1,
					1, 0, 1, 1,
					0, 1, 1, 1,
					1, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation:   OP_Build,
					Expectation: []ExpectationFunc{},
				},
				&TestOperation{
					Operation: OP_GetIntersectedObjects(0),
					Expectation: []ExpectationFunc{EX_CheckIntersectedObjects(
						[]PhysicalObject{
							&TestPhysicalObject{1, 1, 1, 1},
						},
					)},
				},
			},
		},
		&TestCase{ // GetIntersectedObjects, 跨级，从父节点中找Intersection
			Setup: &TestSetup{
				0, 0, 4, 4,
				1, 10,
				[]float32{
					1.5, 1, 1, 1,
					0, 0, 1, 1,
					1, 0, 1, 1,
					0, 1, 1, 1,
					1, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation:   OP_Build,
					Expectation: []ExpectationFunc{},
				},
				&TestOperation{
					Operation: OP_GetIntersectedObjects(4),
					Expectation: []ExpectationFunc{EX_CheckIntersectedObjects(
						[]PhysicalObject{
							&TestPhysicalObject{1.5, 1, 1, 1},
						},
					)},
				},
			},
		},
		&TestCase{ // GetIntersectedObjects, 从父节点和字节点找出多个Intersection
			Setup: &TestSetup{
				0, 0, 4, 4,
				1, 10,
				[]float32{
					1, 1, 2, 2,
					0.5, 0.5, 1, 1,
					0, 1, 1, 1,
					1, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation:   OP_Build,
					Expectation: []ExpectationFunc{},
				},
				&TestOperation{
					Operation: OP_GetIntersectedObjects(1),
					Expectation: []ExpectationFunc{EX_CheckIntersectedObjects(
						[]PhysicalObject{
							&TestPhysicalObject{1, 1, 2, 2},
							&TestPhysicalObject{0, 1, 1, 1},
							&TestPhysicalObject{1, 1, 1, 1},
						},
					)},
				},
			},
		},
		&TestCase{ // FindObject, 叶子节点
			Setup: &TestSetup{
				0, 0, 4, 4,
				1, 10,
				[]float32{
					0, 0, 1, 1,
					1, 0, 1, 1,
					0, 1, 1, 1,
					1, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation:   OP_Build,
					Expectation: []ExpectationFunc{},
				},
				&TestOperation{
					Operation: OP_FindObject(3),
					Expectation: []ExpectationFunc{EX_CheckState(&QuadtreeState{
						[]float32{1, 1, 1, 1},
						[4]*QuadtreeState{},
					})},
				},
			},
		},
		&TestCase{ // FindObject, 非叶子节点
			Setup: &TestSetup{
				0, 0, 4, 4,
				1, 10,
				[]float32{
					0.5, 0.5, 1, 1,
					0, 0, 1, 1,
					1, 0, 1, 1,
					0, 1, 1, 1,
					1, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation:   OP_Build,
					Expectation: []ExpectationFunc{},
				},
				&TestOperation{
					Operation: OP_FindObject(0),
					Expectation: []ExpectationFunc{EX_CheckState(&QuadtreeState{
						[]float32{0.5, 0.5, 1, 1},
						[4]*QuadtreeState{
							&QuadtreeState{
								[]float32{0, 0, 1, 1},
								[4]*QuadtreeState{},
							},
							&QuadtreeState{
								[]float32{1, 0, 1, 1},
								[4]*QuadtreeState{},
							},
							&QuadtreeState{
								[]float32{0, 1, 1, 1},
								[4]*QuadtreeState{},
							},
							&QuadtreeState{
								[]float32{1, 1, 1, 1},
								[4]*QuadtreeState{},
							},
						},
					})},
				},
			},
		},
		&TestCase{ // 拆分的时候，只创建必要的子节点
			Setup: &TestSetup{
				0, 0, 2, 2,
				1, 1,
				[]float32{
					0, 0, 1, 1, // top-left subnode
					1, 0, 1, 1, // top-right subnode
					0, 1, 1, 1, // bottom-left subnode
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{},
							[4]*QuadtreeState{
								&QuadtreeState{[]float32{0, 0, 1, 1}, [4]*QuadtreeState{}}, // top-left subnode
								&QuadtreeState{[]float32{1, 0, 1, 1}, [4]*QuadtreeState{}}, // top-right subnode
								&QuadtreeState{[]float32{0, 1, 1, 1}, [4]*QuadtreeState{}}, // bottom-left subnode
								nil, // no bottom-right subnode
							},
						},
					)},
				},
			},
		},
		&TestCase{ // 最多拆分MaxLevels次 (对比之一: 拆分两次)
			Setup: &TestSetup{
				0, 0, 4, 4,
				1, 2,
				[]float32{
					1.5, 1.5, 1, 1,
					0, 0, 1, 1,
					1, 0, 1, 1,
					0, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{1.5, 1.5, 1, 1},
							[4]*QuadtreeState{
								&QuadtreeState{
									[]float32{},
									[4]*QuadtreeState{
										&QuadtreeState{[]float32{0, 0, 1, 1}, [4]*QuadtreeState{}},
										&QuadtreeState{[]float32{1, 0, 1, 1}, [4]*QuadtreeState{}},
										&QuadtreeState{[]float32{0, 1, 1, 1}, [4]*QuadtreeState{}},
										nil,
									},
								}, // top-left subnode
								nil,
								nil,
								nil,
							},
						},
					)},
				},
			},
		},
		&TestCase{ // 最多拆分MaxLevels次 (对比之一: 拆分一次)
			Setup: &TestSetup{
				0, 0, 4, 4,
				1, 1,
				[]float32{
					1.5, 1.5, 1, 1,
					0, 0, 1, 1,
					1, 0, 1, 1,
					0, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{
								1.5, 1.5, 1, 1,
							},
							[4]*QuadtreeState{
								&QuadtreeState{
									[]float32{
										0, 0, 1, 1,
										1, 0, 1, 1,
										0, 1, 1, 1,
									},
									[4]*QuadtreeState{},
								},
								nil,
								nil,
								nil,
							},
						},
					)},
				},
			},
		},
		&TestCase{ // 物体数量不超过MaxObjects(对比之一: 不拆分)
			Setup: &TestSetup{
				0, 0, 2, 2,
				4, 1,
				[]float32{
					0, 0, 1, 1, // top-left subnode
					1, 0, 1, 1, // top-right subnode
					0, 1, 1, 1, // bottom-left subnode
					1, 1, 1, 1, // bottom-right subnode
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{
								0, 0, 1, 1,
								1, 0, 1, 1,
								0, 1, 1, 1,
								1, 1, 1, 1,
							},
							[4]*QuadtreeState{},
						},
					)},
				},
			},
		},
		&TestCase{ // 物体数量不超过MaxObjects(对比之二: 拆分)
			Setup: &TestSetup{
				0, 0, 2, 2,
				3, 1,
				[]float32{
					0, 0, 1, 1,
					1, 0, 1, 1,
					0, 1, 1, 1,
					1, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{},
							[4]*QuadtreeState{
								&QuadtreeState{[]float32{0, 0, 1, 1}, [4]*QuadtreeState{}},
								&QuadtreeState{[]float32{1, 0, 1, 1}, [4]*QuadtreeState{}},
								&QuadtreeState{[]float32{0, 1, 1, 1}, [4]*QuadtreeState{}},
								&QuadtreeState{[]float32{1, 1, 1, 1}, [4]*QuadtreeState{}},
							},
						},
					)},
				},
			},
		},
		&TestCase{ // 拆分3次
			Setup: &TestSetup{
				0, 0, 8, 8,
				1, 5,
				[]float32{
					3.5, 3.5, 1, 1,
					1.5, 1.5, 1, 1,
					0, 0, 1, 1,
					1, 0, 1, 1,
					0, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{3.5, 3.5, 1, 1},
							[4]*QuadtreeState{
								&QuadtreeState{
									[]float32{1.5, 1.5, 1, 1},
									[4]*QuadtreeState{
										&QuadtreeState{
											[]float32{},
											[4]*QuadtreeState{
												&QuadtreeState{
													[]float32{0, 0, 1, 1},
													[4]*QuadtreeState{},
												},
												&QuadtreeState{
													[]float32{1, 0, 1, 1},
													[4]*QuadtreeState{},
												},
												&QuadtreeState{
													[]float32{0, 1, 1, 1},
													[4]*QuadtreeState{},
												},
											},
										},
									},
								},
							},
						},
					)},
				},
			},
		},
		&TestCase{ // 插入新节点，存储在非叶子节点
			Setup: &TestSetup{
				0, 0, 4, 4,
				1, 10,
				[]float32{
					1.5, 1.5, 1, 1,
					0.5, 0.5, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{1.5, 1.5, 1, 1},
							[4]*QuadtreeState{
								&QuadtreeState{[]float32{0.5, 0.5, 1, 1}, [4]*QuadtreeState{}},
							},
						},
					)},
				},
				&TestOperation{
					Operation: OP_Insert(
						3, 1.5, 1, 1,
					),
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{
								1.5, 1.5, 1, 1,
								3, 1.5, 1, 1,
							},
							[4]*QuadtreeState{
								&QuadtreeState{[]float32{0.5, 0.5, 1, 1}, [4]*QuadtreeState{}},
							},
						},
					)},
				},
			},
		},
		&TestCase{ // 插入新物体，使原本的叶子节点被拆分
			Setup: &TestSetup{
				0, 0, 4, 4,
				1, 10,
				[]float32{
					1.5, 1.5, 1, 1,
					0, 0, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{1.5, 1.5, 1, 1},
							[4]*QuadtreeState{
								&QuadtreeState{[]float32{0, 0, 1, 1}, [4]*QuadtreeState{}},
							},
						},
					)},
				},
				&TestOperation{
					Operation: OP_Insert(
						0, 1, 1, 1,
					),
					Expectation: []ExpectationFunc{EX_CheckState(
						&QuadtreeState{
							[]float32{1.5, 1.5, 1, 1},
							[4]*QuadtreeState{
								&QuadtreeState{
									[]float32{},
									[4]*QuadtreeState{
										&QuadtreeState{
											[]float32{0, 0, 1, 1},
											[4]*QuadtreeState{},
										},
										nil,
										&QuadtreeState{
											[]float32{0, 1, 1, 1},
											[4]*QuadtreeState{},
										},
										nil,
									},
								},
							},
						},
					)},
				},
			},
		},
		&TestCase{ // 插入新物体，导致原本的叶子节点尝试进行拆分，但是由于新对象位置的关系并没有创建新的子节点
			Setup: &TestSetup{
				0, 0, 4, 4,
				1, 10,
				[]float32{
					1.5, 1.5, 1, 1,
					0, 0.5, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckStateAndIntersections(&TestExpectation{
						&QuadtreeState{
							[]float32{1.5, 1.5, 1, 1},
							[4]*QuadtreeState{
								&QuadtreeState{[]float32{0, 0.5, 1, 1}, [4]*QuadtreeState{}},
							},
						},
						nil,
					})},
				},
				&TestOperation{
					Operation: OP_Insert(
						1, 0.5, 1, 1,
					),
					Expectation: []ExpectationFunc{EX_CheckStateAndIntersections(&TestExpectation{
						&QuadtreeState{
							[]float32{
								1.5, 1.5, 1, 1,
							},
							[4]*QuadtreeState{
								&QuadtreeState{
									[]float32{
										0, 0.5, 1, 1,
										1, 0.5, 1, 1,
									},
									[4]*QuadtreeState{},
								},
							},
						},
						nil,
					})},
				},
			},
		},
		&TestCase{ // 物体之间只是接触，没有交叉，不算碰撞
			Setup: &TestSetup{
				0, 0, 2, 2,
				4, 1,
				[]float32{
					0, 0, 1, 1,
					1, 0, 1, 1,
					0, 1, 1, 1,
					1, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckStateAndIntersections(&TestExpectation{
						&QuadtreeState{
							[]float32{
								0, 0, 1, 1,
								1, 0, 1, 1,
								0, 1, 1, 1,
								1, 1, 1, 1,
							},
							[4]*QuadtreeState{},
						},
						nil,
					})},
				},
			},
		},
		&TestCase{ // 父节点与字节点之间碰撞
			Setup: &TestSetup{
				0, 0, 2, 2,
				1, 1,
				[]float32{
					0.5, 0.5, 1, 1,
					0, 0, 1, 1,
					1, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckStateAndIntersections(&TestExpectation{
						&QuadtreeState{
							[]float32{
								0.5, 0.5, 1, 1,
							},
							[4]*QuadtreeState{
								&QuadtreeState{
									[]float32{
										0, 0, 1, 1,
									},
									[4]*QuadtreeState{},
								},
								nil,
								nil,
								&QuadtreeState{
									[]float32{
										1, 1, 1, 1,
									},
									[4]*QuadtreeState{},
								},
							},
						},
						[]PhysicalObject{
							&TestPhysicalObject{0.5, 0.5, 1, 1},
							&TestPhysicalObject{0, 0, 1, 1},
							&TestPhysicalObject{0.5, 0.5, 1, 1},
							&TestPhysicalObject{1, 1, 1, 1},
						},
					})},
				},
			},
		},
		&TestCase{ // 同(0)级节点之间碰撞
			Setup: &TestSetup{
				0, 0, 2, 2,
				4, 1,
				[]float32{
					0.5, 0.5, 1, 1,
					0, 0, 1, 1,
					1, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckStateAndIntersections(&TestExpectation{
						&QuadtreeState{
							[]float32{
								0.5, 0.5, 1, 1,
								0, 0, 1, 1,
								1, 1, 1, 1,
							},
							[4]*QuadtreeState{},
						},
						[]PhysicalObject{
							&TestPhysicalObject{0.5, 0.5, 1, 1},
							&TestPhysicalObject{0, 0, 1, 1},
							&TestPhysicalObject{0.5, 0.5, 1, 1},
							&TestPhysicalObject{1, 1, 1, 1},
						},
					})},
				},
			},
		},
		&TestCase{ // 跨级(0 vs. 2)碰撞
			Setup: &TestSetup{
				0, 0, 4, 4,
				1, 10,
				[]float32{
					1.5, 1.5, 1, 1,
					0, 0, 1, 1,
					1, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckStateAndIntersections(&TestExpectation{
						&QuadtreeState{
							[]float32{
								1.5, 1.5, 1, 1,
							},
							[4]*QuadtreeState{
								&QuadtreeState{
									[]float32{},
									[4]*QuadtreeState{
										&QuadtreeState{
											[]float32{
												0, 0, 1, 1,
											},
											[4]*QuadtreeState{},
										},
										nil,
										nil,
										&QuadtreeState{
											[]float32{
												1, 1, 1, 1,
											},
											[4]*QuadtreeState{},
										},
									},
								},
								nil,
								nil,
								nil,
							},
						},
						[]PhysicalObject{
							&TestPhysicalObject{1.5, 1.5, 1, 1},
							&TestPhysicalObject{1, 1, 1, 1},
						},
					})},
				},
			},
		},
		&TestCase{ // 同(2)级以及父子节点(1 vs. 2)以及跨级(0 vs. 2)同时碰撞
			Setup: &TestSetup{
				0, 0, 4, 4,
				1, 10,
				[]float32{
					1.5, 1.5, 1, 1,
					0, 0, 1, 1,
					0, 0, 1, 1,
					0.5, 0, 1, 1,
					1, 1, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckStateAndIntersections(&TestExpectation{
						&QuadtreeState{
							[]float32{
								1.5, 1.5, 1, 1,
							},
							[4]*QuadtreeState{
								&QuadtreeState{
									[]float32{
										0.5, 0, 1, 1,
									},
									[4]*QuadtreeState{
										&QuadtreeState{
											[]float32{
												0, 0, 1, 1,
												0, 0, 1, 1,
											},
											[4]*QuadtreeState{},
										},
										nil,
										nil,
										&QuadtreeState{
											[]float32{1, 1, 1, 1},
											[4]*QuadtreeState{},
										},
									},
								},
								nil,
								nil,
								nil,
							},
						},
						[]PhysicalObject{
							// 0 vs. 2
							&TestPhysicalObject{1.5, 1.5, 1, 1},
							&TestPhysicalObject{1, 1, 1, 1},

							// 2 vs. 2
							&TestPhysicalObject{0, 0, 1, 1},
							&TestPhysicalObject{0, 0, 1, 1},

							// 1 vs. 2
							&TestPhysicalObject{0, 0, 1, 1},
							&TestPhysicalObject{0.5, 0, 1, 1},

							&TestPhysicalObject{0, 0, 1, 1},
							&TestPhysicalObject{0.5, 0, 1, 1},
						},
					})},
				},
			},
		},
		&TestCase{
			Setup: &TestSetup{
				0, 0, 2, 2,
				1, 10,
				[]float32{
					0, 0, 1, 1,
					1, 0, 1, 1,
				},
			},
			Operations: []*TestOperation{
				&TestOperation{
					Operation: OP_Build,
					Expectation: []ExpectationFunc{EX_CheckStateAndIntersections(&TestExpectation{
						&QuadtreeState{
							[]float32{},
							[4]*QuadtreeState{
								&QuadtreeState{
									[]float32{0, 0, 1, 1},
									[4]*QuadtreeState{},
								},
								&QuadtreeState{
									[]float32{1, 0, 1, 1},
									[4]*QuadtreeState{},
								},
								nil,
								nil,
							},
						},
						nil,
					})},
				},
				// 物体移动，创建新的节点
				&TestOperation{
					Operation: OP_UpdateObject(0, 0, 1, 1),
					Expectation: []ExpectationFunc{EX_CheckStateAndIntersections(&TestExpectation{
						&QuadtreeState{
							[]float32{},
							[4]*QuadtreeState{
								&QuadtreeState{}, // 旧的节点保留，但是没有物理对象
								&QuadtreeState{
									[]float32{1, 0, 1, 1},
									[4]*QuadtreeState{},
								},
								&QuadtreeState{
									[]float32{0, 1, 1, 1},
									[4]*QuadtreeState{},
								},
								nil,
							},
						},
						nil,
					})},
				},
				// 再Update若干次，空节点仍在
				&TestOperation{
					Operation: OP_UpdateObject(0, 0, 1, 63),
					Expectation: []ExpectationFunc{EX_CheckStateAndIntersections(&TestExpectation{
						&QuadtreeState{
							[]float32{},
							[4]*QuadtreeState{
								&QuadtreeState{}, // 旧的节点保留，但是没有物理对象
								&QuadtreeState{
									[]float32{1, 0, 1, 1},
									[4]*QuadtreeState{},
								},
								&QuadtreeState{
									[]float32{0, 1, 1, 1},
									[4]*QuadtreeState{},
								},
								nil,
							},
						},
						nil,
					})},
				},
				// 再Update一次，空节点删除
				&TestOperation{
					Operation: OP_UpdateObject(0, 0, 1, 1),
					Expectation: []ExpectationFunc{EX_CheckStateAndIntersections(&TestExpectation{
						&QuadtreeState{
							[]float32{},
							[4]*QuadtreeState{
								nil,
								&QuadtreeState{
									[]float32{1, 0, 1, 1},
									[4]*QuadtreeState{},
								},
								&QuadtreeState{
									[]float32{0, 1, 1, 1},
									[4]*QuadtreeState{},
								},
								nil,
							},
						},
						nil,
					})},
				},
			},
		},
	}
)

func TestAll(t *testing.T) {
	for testIndex, testCase := range testCases {
		// setup
		var objects []PhysicalObject
		for i := 0; i < len(testCase.Setup.PhysicalObjects); i += 4 {
			objects = append(objects, &TestPhysicalObject{
				X:      testCase.Setup.PhysicalObjects[i],
				Y:      testCase.Setup.PhysicalObjects[i+1],
				Width:  testCase.Setup.PhysicalObjects[i+2],
				Height: testCase.Setup.PhysicalObjects[i+3],
			})
		}
		qt := CreateQuadtree(
			&Bounds{testCase.Setup.X, testCase.Setup.Y, testCase.Setup.Width, testCase.Setup.Height},
			testCase.Setup.MaxObjects,
			testCase.Setup.MaxLevels,
			objects...,
		)

		// do operation and evaluate expectation
		for _, op := range testCase.Operations {
			results := op.Operation(qt, objects)
			for _, ex := range op.Expectation {
				ex(t, testIndex, results)
			}
		}
	}
}
