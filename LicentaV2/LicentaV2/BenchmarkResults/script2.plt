set terminal png size 1024,768
set print "-"
set term png font "arial, 14"
set border lw 1.5

replace(S,C,R)=(strstrt(S,C)) ? replace( S[:strstrt(S,C)-1].R.S[strstrt(S,C)+strlen(C):] ,C,R) : S


method_names = "BVH Octree Spatial-Grid Spatial-Hashing Sweep-and-Prune"
criterion_names = "Average_Time_Spent_-_Structure_Update(ms) Average_Intersection_Tests Average_Min_Memory_Used(KB) Average_Max_Memory_Used(KB) Average_Time_Spent_-_Collisions(ms)"
y_label_names = "Average_Time_(ms) Average_Memory(KB) Average_Time_(ms) Average_Tests Average_Memory(KB)"
number_of_scenarios = 4

filename(number, method_name) = sprintf("RawResults\\Scenario_%d_average_%s.txt", number, method_name)

outputName(number, criterionName) = sprintf("Plots\\PerScenario\\Scenario_%d_average_%s.png", number, criterionName)

titleName(number, criterionName) = sprintf("Scenario %d: %s", number, replace(criterionName, "_", " "))

prt(number, criterionName) = sprintf("%d %s", number, criterionName)

do for [i = 0:number_of_scenarios] {
	criterionIndex = 2
	do for [criterion in criterion_names] {
		humanReadableCriterion = replace(criterion, "_", " ")		
		#print prt(criterionIndex, humanReadableCriterion)
		set output outputName(i, humanReadableCriterion)
		set title titleName(i, criterion)
		set xlabel "Number of Objects"
		set ylabel humanReadableCriterion
		#set key below
		#set size 1,2
		plot for [method in method_names] filename(i, method) using 1:criterionIndex with lines title replace(method, "-", " ") lw 2
		criterionIndex = criterionIndex + 1
	}
}