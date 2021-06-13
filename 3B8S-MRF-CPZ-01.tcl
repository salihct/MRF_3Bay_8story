# --------------------------------------------------------------------------------------------------
# Example: 2-Story Steel Moment Frame with Concentrated Plasticity
# Panel Zone Model with Concentrated Plastic Hinges at RBS Locations
# Created by:  Laura Eads, Stanford University, 2010
# Units: kips, inches, seconds

# Element ID conventions:
#	1xy    = frame columns with RBS springs at both ends
#	2xy    = frame beams with RBS springs at both ends
#	6xy    = trusses linking frame and P-delta column
#	7xy    = P-delta columns
#	2xya   = frame beams between panel zone and RBS spring
#	3xya   = frame column rotational springs
#	4xya   = frame beam rotational springs
#	5xya   = P-delta column rotational springs
#	4xy00  = panel zone rotational springs
#	500xya = panel zone elements
#	where:
#		x = Pier or Bay #
#		y = Floor or Story #
#		a = an integer describing the location relative to beam-column joint (see description where elements and nodes are defined)

###################################################################################################
#          Set Up & Source Definition									  
###################################################################################################
	wipe all;							# clear memory of past model definitions
	model BasicBuilder -ndm 2 -ndf 3;	# Define the model builder, ndm = #dimension, ndf = #dofs
	source DisplayModel2D.tcl;			# procedure for displaying a 2D perspective of model
	source DisplayPlane.tcl;			# procedure for displaying a plane in a model
	source rotSpring2DModIKModel.tcl;	# procedure for defining a rotational spring (zero-length element) for plastic hinges
	source rotLeaningCol.tcl;			# procedure for defining a rotational spring (zero-length element) with very small stiffness
	source rotPanelZone2D.tcl;			# procedure for defining a rotational spring (zero-length element) to capture panel zone shear distortions
	source elemPanelZone2D.tcl;			# procedure for defining 8 elements to create a rectangular panel zone
	
###################################################################################################
#          Define Analysis Type										  
###################################################################################################
# Define type of analysis:  "pushover" = pushover;  "dynamic" = dynamic
	set analysisType "pushover";
	
	if {$analysisType == "pushover"} {
		set dataDir Concentrated-PanelZone-Pushover-Output;	# name of output folder
		file mkdir $dataDir; 								# create output folder
	}
	if {$analysisType == "dynamic"} {
		set dataDir Concentrated-PanelZone-Dynamic-Output;	# name of output folder
		file mkdir $dataDir; 								# create output folder
	}
	
###################################################################################################
#          Define Building Geometry, Nodes, Masses, and Constraints											  
###################################################################################################
# define structure-geometry parameters
	set NStories 8;						# number of stories
	set NBays 3;						# number of frame bays (excludes bay for P-delta column)
	set WBay      [expr 20.01*12.0];		# bay width in inches
	set HStory1   [expr 15.09*12.0];		# 1st story height in inches
	set HStoryTyp [expr 13.12*12.0];		# story height of other stories in inches
	set HBuilding [expr $HStory1 + ($NStories-1)*$HStoryTyp];	# height of building

# calculate locations of beam-column joint centerlines:
	set Pier1  0.0;		# leftmost column line
	set Pier2  [expr $Pier1 + $WBay];
	set Pier3  [expr $Pier2 + $WBay];	
	set Pier4  [expr $Pier3 + $WBay];
	set Pier5  [expr $Pier4 + $WBay];  # P-delta column line
	
	set Floor1 0.0;		# ground floor
	set Floor2 [expr $Floor1 + $HStory1];
	set Floor3 [expr $Floor2 + $HStoryTyp];
	set Floor4 [expr $Floor3 + $HStoryTyp];
	set Floor5 [expr $Floor4 + $HStoryTyp];
	set Floor6 [expr $Floor5 + $HStoryTyp];
	set Floor7 [expr $Floor6 + $HStoryTyp];
	set Floor8 [expr $Floor7 + $HStoryTyp];
	set Floor9 [expr $Floor8 + $HStoryTyp];
	
# calculate panel zone dimensions
	set pzlat1423   [expr 24.74/2.0];	# lateral dist from CL of beam-col joint to edge of panel zone (= half the column depth)  pzlatx1x2y1y2 , x1=peir x2=peir y1=floor y2=floor
	set pzlat1445   [expr 24.48/2.0];	# pzlatx1x2y1y2 , x1=peir x2=peir y1=floor y2=floor
	set pzlat1467   [expr 24.31/2.0];	# pzlatx1x2y1y2 , x1=peir x2=peir y1=floor y2=floor
	set pzlat1489   [expr 23.92/2.0];	# pzlatx1x2y1y2 , x1=peir x2=peir y1=floor y2=floor
	
	set pzlat2323   [expr 25/2.0];	# lateral dist from CL of beam-col joint to edge of panel zone (= half the column depth)  pzlatx1x2y1y2 , x1=peir x2=peir y1=floor y2=floor
	set pzlat2345   [expr 25/2.0];	# pzlatx1x2y1y2 , x1=peir x2=peir y1=floor y2=floor
	set pzlat2367   [expr 24.48/2.0];	# pzlatx1x2y1y2 , x1=peir x2=peir y1=floor y2=floor
	set pzlat2389   [expr 23.92/2.0];	# pzlatx1x2y1y2 , x1=peir x2=peir y1=floor y2=floor
	
	set pzvert23 [expr 26.92/2.0];	# vert dist from CL of beam-col joint to edge of panel zone (= half the beam depth) pzvertx1x2 ,x1=floor x2=floor
	set pzvert45 [expr 24.31/2.0];
	set pzvert67 [expr 23.92/2.0];
	set pzvert89 [expr 17.99/2.0];
	
# calculate plastic hinge offsets from beam-column centerlines:
	set phlat1423 [expr $pzlat1423  + 7.5 + 22.5/2.0];	# lateral dist from CL of beam-col joint to beam hinge
	set phlat1445 [expr $pzlat1445  + 7.5 + 22.5/2.0];
	set phlat1467 [expr $pzlat1467  + 7.5 + 22.5/2.0];	
	set phlat1489 [expr $pzlat1489  + 7.5 + 22.5/2.0];
	
	set phlat2323 [expr $pzlat2323  + 7.5 + 22.5/2.0];	# lateral dist from CL of beam-col joint to beam hinge
	set phlat2345 [expr $pzlat2345  + 7.5 + 22.5/2.0];
	set phlat2367 [expr $pzlat2367  + 7.5 + 22.5/2.0];	
	set phlat2389 [expr $pzlat2389  + 7.5 + 22.5/2.0];
	
	set phvert23 [expr $pzvert23 + 0.0];			# vert dist from CL of beam-col joint to column hinge (forms at edge of panel zone)
	set phvert45 [expr $pzvert45 + 0.0];
	set phvert67 [expr $pzvert67 + 0.0];
	set phvert89 [expr $pzvert89 + 0.0];

# calculate nodal masses -- lump floor masses at frame nodes
	set g 386.2;				# acceleration due to gravity
	set FloorWeight 586.25;		# weight of all Floor  in kips
	#set Floor3Weight 525.0;		# weight of Floor 3 in kips
	#set Floor4Weight 525.0;		# weight of Floor 4 in kips
	set WBuilding  [expr $FloorWeight * $NStories]; # total building weight
	set NodalMass [expr ($FloorWeight/$g) / (4.0)];	# mass at each node on Floor 2
	#set NodalMass3 [expr ($Floor3Weight/$g) / (2.0)];	# mass at each node on Floor 3
	#set NodalMass4 [expr ($Floor4Weight/$g) / (2.0)];	# mass at each node on Floor 4
	set Negligible 1e-9;	# a very small number to avoid problems with zero

# define nodes and assign masses to beam-column intersections of frame
	# command:  node nodeID xcoord ycoord -mass mass_dof1 mass_dof2 mass_dof3
	# nodeID convention:  "xy" where x = Pier # and y = Floor # 
	node 11 $Pier1 $Floor1;
	node 21 $Pier2 $Floor1;
	node 31 $Pier3 $Floor1;
	node 41 $Pier4 $Floor1;
	node 51 $Pier5 $Floor1;
	node 52 $Pier5 $Floor2;
	node 53 $Pier5 $Floor3;
	node 54 $Pier5 $Floor4;
	node 55 $Pier5 $Floor5;
	node 56 $Pier5 $Floor6;
	node 57 $Pier5 $Floor7;
	node 58 $Pier5 $Floor8;
	node 59 $Pier5 $Floor9;
	

# define extra nodes for plastic hinge rotational springs
	# nodeID convention:  "xya" where x = Pier #, y = Floor #, a = location relative to beam-column joint
	# "a" convention: 1,2 = left; 3,4 = right; (used for beams)
	# "a" convention: 5,6 = below; 7,8 = above; (used for columns)
	# column hinges at bottom of Story 1 (base)
	node 117 $Pier1 $Floor1;
	node 217 $Pier2 $Floor1;
	node 317 $Pier3 $Floor1;
	node 417 $Pier4 $Floor1;
	# column hinges at top of Story 1
	node 125 $Pier1 [expr $Floor2 - $phvert23];
	node 126 $Pier1 [expr $Floor2 - $phvert23];
	node 225 $Pier2 [expr $Floor2 - $phvert23];
	node 226 $Pier2 [expr $Floor2 - $phvert23];
	node 325 $Pier3 [expr $Floor2 - $phvert23];
	node 326 $Pier3 [expr $Floor2 - $phvert23];
	node 425 $Pier4 [expr $Floor2 - $phvert23];
	node 426 $Pier4 [expr $Floor2 - $phvert23];
	node 526 $Pier5 $Floor2;	# zero-stiffness spring will be used on p-delta column
	# column hinges at bottom of Story 2
	node 127 $Pier1 [expr $Floor2 + $phvert23];
	node 128 $Pier1 [expr $Floor2 + $phvert23];
	node 227 $Pier2 [expr $Floor2 + $phvert23];
	node 228 $Pier2 [expr $Floor2 + $phvert23];
	node 327 $Pier3 [expr $Floor2 + $phvert23];
	node 328 $Pier3 [expr $Floor2 + $phvert23];
	node 427 $Pier4 [expr $Floor2 + $phvert23];
	node 428 $Pier4 [expr $Floor2 + $phvert23];
	node 527 $Pier5 $Floor2;	# zero-stiffness spring will be used on p-delta column
	# column hinges at top of Story 2
	node 135 $Pier1 [expr $Floor3 - $phvert23];
	node 136 $Pier1 [expr $Floor3 - $phvert23];
	node 235 $Pier2 [expr $Floor3 - $phvert23];
	node 236 $Pier2 [expr $Floor3 - $phvert23];
	node 335 $Pier3 [expr $Floor3 - $phvert23];
	node 336 $Pier3 [expr $Floor3 - $phvert23];
	node 435 $Pier4 [expr $Floor3 - $phvert23];
	node 436 $Pier4 [expr $Floor3 - $phvert23];
	node 536 $Pier5 $Floor3;	# zero-stiffness spring will be used on p-delta column
	# column hinges at bottom of Story 3
	node 137 $Pier1 [expr $Floor3 + $phvert23];
	node 138 $Pier1 [expr $Floor3 + $phvert23];
	node 237 $Pier2 [expr $Floor3 + $phvert23];
	node 238 $Pier2 [expr $Floor3 + $phvert23];
	node 337 $Pier3 [expr $Floor3 + $phvert23];
	node 338 $Pier3 [expr $Floor3 + $phvert23];
	node 437 $Pier4 [expr $Floor3 + $phvert23];
	node 438 $Pier4 [expr $Floor3 + $phvert23];
	node 537 $Pier5 $Floor3;	# zero-stiffness spring will be used on p-delta column
	# column hinges at top of Story 3
	node 145 $Pier1 [expr $Floor4 - $phvert45];
	node 146 $Pier1 [expr $Floor4 - $phvert45];
	node 245 $Pier2 [expr $Floor4 - $phvert45];
	node 246 $Pier2 [expr $Floor4 - $phvert45];
	node 345 $Pier3 [expr $Floor4 - $phvert45];
	node 346 $Pier3 [expr $Floor4 - $phvert45];
	node 445 $Pier4 [expr $Floor4 - $phvert45];
	node 446 $Pier4 [expr $Floor4 - $phvert45];
	node 546 $Pier5 $Floor4;	# zero-stiffness spring will be used on p-delta column
	# column hinges at bottom of Story 4
	node 147 $Pier1 [expr $Floor4 + $phvert45];
	node 148 $Pier1 [expr $Floor4 + $phvert45];
	node 247 $Pier2 [expr $Floor4 + $phvert45];
	node 248 $Pier2 [expr $Floor4 + $phvert45];
	node 347 $Pier3 [expr $Floor4 + $phvert45];
	node 348 $Pier3 [expr $Floor4 + $phvert45];
	node 447 $Pier4 [expr $Floor4 + $phvert45];
	node 448 $Pier4 [expr $Floor4 + $phvert45];
	node 547 $Pier5 $Floor4;	# zero-stiffness spring will be used on p-delta column
	# column hinges at top of Story 4
	node 155 $Pier1 [expr $Floor5 - $phvert45];
	node 156 $Pier1 [expr $Floor5 - $phvert45];
	node 255 $Pier2 [expr $Floor5 - $phvert45];
	node 256 $Pier2 [expr $Floor5 - $phvert45];
	node 355 $Pier3 [expr $Floor5 - $phvert45];
	node 356 $Pier3 [expr $Floor5 - $phvert45];
	node 455 $Pier4 [expr $Floor5 - $phvert45];
	node 456 $Pier4 [expr $Floor5 - $phvert45];
	node 556 $Pier5 $Floor5;	# zero-stiffness spring will be used on p-delta column
	# column hinges at bottom of Story 5
	node 157 $Pier1 [expr $Floor5 + $phvert45];
	node 158 $Pier1 [expr $Floor5 + $phvert45];
	node 257 $Pier2 [expr $Floor5 + $phvert45];
	node 258 $Pier2 [expr $Floor5 + $phvert45];
	node 357 $Pier3 [expr $Floor5 + $phvert45];
	node 358 $Pier3 [expr $Floor5 + $phvert45];
	node 457 $Pier4 [expr $Floor5 + $phvert45];
	node 458 $Pier4 [expr $Floor5 + $phvert45];
	node 557 $Pier5 $Floor5;	# zero-stiffness spring will be used on p-delta column
	# column hinges at top of Story 5
	node 165 $Pier1 [expr $Floor6 - $phvert67];
	node 166 $Pier1 [expr $Floor6 - $phvert67];
	node 265 $Pier2 [expr $Floor6 - $phvert67];
	node 266 $Pier2 [expr $Floor6 - $phvert67];
	node 365 $Pier3 [expr $Floor6 - $phvert67];
	node 366 $Pier3 [expr $Floor6 - $phvert67];
	node 465 $Pier4 [expr $Floor6 - $phvert67];
	node 466 $Pier4 [expr $Floor6 - $phvert67];
	node 566 $Pier5 $Floor6;	# zero-stiffness spring will be used on p-delta column
	# column hinges at bottom of Story 6
	node 167 $Pier1 [expr $Floor6 + $phvert67];
	node 168 $Pier1 [expr $Floor6 + $phvert67];
	node 267 $Pier2 [expr $Floor6 + $phvert67];
	node 268 $Pier2 [expr $Floor6 + $phvert67];
	node 367 $Pier3 [expr $Floor6 + $phvert67];
	node 368 $Pier3 [expr $Floor6 + $phvert67];
	node 467 $Pier4 [expr $Floor6 + $phvert67];
	node 468 $Pier4 [expr $Floor6 + $phvert67];
	node 567 $Pier5 $Floor6;	# zero-stiffness spring will be used on p-delta column
	# column hinges at top of Story 6
	node 175 $Pier1 [expr $Floor7 - $phvert67];
	node 176 $Pier1 [expr $Floor7 - $phvert67];
	node 275 $Pier2 [expr $Floor7 - $phvert67];
	node 276 $Pier2 [expr $Floor7 - $phvert67];
	node 375 $Pier3 [expr $Floor7 - $phvert67];
	node 376 $Pier3 [expr $Floor7 - $phvert67];
	node 475 $Pier4 [expr $Floor7 - $phvert67];
	node 476 $Pier4 [expr $Floor7 - $phvert67];
	node 576 $Pier5 $Floor7;	# zero-stiffness spring will be used on p-delta columnS
	# column hinges at bottom of Story 7
	node 177 $Pier1 [expr $Floor7 + $phvert67];
	node 178 $Pier1 [expr $Floor7 + $phvert67];
	node 277 $Pier2 [expr $Floor7 + $phvert67];
	node 278 $Pier2 [expr $Floor7 + $phvert67];
	node 377 $Pier3 [expr $Floor7 + $phvert67];
	node 378 $Pier3 [expr $Floor7 + $phvert67];
	node 477 $Pier4 [expr $Floor7 + $phvert67];
	node 478 $Pier4 [expr $Floor7 + $phvert67];
	node 577 $Pier5 $Floor7;	# zero-stiffness spring will be used on p-delta column
	# column hinges at top of Story 7
	node 185 $Pier1 [expr $Floor8 - $phvert89];
	node 186 $Pier1 [expr $Floor8 - $phvert89];
	node 285 $Pier2 [expr $Floor8 - $phvert89];
	node 286 $Pier2 [expr $Floor8 - $phvert89];
	node 385 $Pier3 [expr $Floor8 - $phvert89];
	node 386 $Pier3 [expr $Floor8 - $phvert89];
	node 485 $Pier4 [expr $Floor8 - $phvert89];
	node 486 $Pier4 [expr $Floor8 - $phvert89];
	node 586 $Pier5 $Floor8;	# zero-stiffness spring will be used on p-delta column
	# column hinges at bottom of Story 8
	node 187 $Pier1 [expr $Floor8 + $phvert89];
	node 188 $Pier1 [expr $Floor8 + $phvert89];
	node 287 $Pier2 [expr $Floor8 + $phvert89];
	node 288 $Pier2 [expr $Floor8 + $phvert89];
	node 387 $Pier3 [expr $Floor8 + $phvert89];
	node 388 $Pier3 [expr $Floor8 + $phvert89];
	node 487 $Pier4 [expr $Floor8 + $phvert89];
	node 488 $Pier4 [expr $Floor8 + $phvert89];
	node 587 $Pier5 $Floor8;	# zero-stiffness spring will be used on p-delta column
	# column hinges at top of Story 8
	node 195 $Pier1 [expr $Floor9 - $phvert89];
	node 196 $Pier1 [expr $Floor9 - $phvert89];
	node 295 $Pier2 [expr $Floor9 - $phvert89];
	node 296 $Pier2 [expr $Floor9 - $phvert89];
	node 395 $Pier3 [expr $Floor9 - $phvert89];
	node 396 $Pier3 [expr $Floor9 - $phvert89];
	node 495 $Pier4 [expr $Floor9 - $phvert89];
	node 496 $Pier4 [expr $Floor9 - $phvert89];
	node 596 $Pier5 $Floor9;	# zero-stiffness spring will be used on p-delta column

	# beam hinges at Floor 2
	node 121 [expr $Pier1 + $phlat1423] $Floor2;
	node 122 [expr $Pier1 + $phlat1423] $Floor2;
	node 223 [expr $Pier2 - $phlat2323] $Floor2;
	node 224 [expr $Pier2 - $phlat2323] $Floor2;
	node 221 [expr $Pier2 + $phlat2323] $Floor2;
	node 222 [expr $Pier2 + $phlat2323] $Floor2;
	node 323 [expr $Pier3 - $phlat2323] $Floor2;
	node 324 [expr $Pier3 - $phlat2323] $Floor2;
	node 321 [expr $Pier3 + $phlat2323] $Floor2;
	node 322 [expr $Pier3 + $phlat2323] $Floor2;
	node 423 [expr $Pier4 - $phlat1423] $Floor2;
	node 424 [expr $Pier4 - $phlat1423] $Floor2;
	# beam hinges at Floor 3
	node 131 [expr $Pier1 + $phlat1423] $Floor3;
	node 132 [expr $Pier1 + $phlat1423] $Floor3;
	node 233 [expr $Pier2 - $phlat2323] $Floor3;
	node 234 [expr $Pier2 - $phlat2323] $Floor3;
	node 231 [expr $Pier2 + $phlat2323] $Floor3;
	node 232 [expr $Pier2 + $phlat2323] $Floor3;
	node 333 [expr $Pier3 - $phlat2323] $Floor3;
	node 334 [expr $Pier3 - $phlat2323] $Floor3;
	node 331 [expr $Pier3 + $phlat2323] $Floor3;
	node 332 [expr $Pier3 + $phlat2323] $Floor3;
	node 433 [expr $Pier4 - $phlat1423] $Floor3;
	node 434 [expr $Pier4 - $phlat1423] $Floor3;
	# beam hinges at Floor 4
	node 141 [expr $Pier1 + $phlat1445] $Floor4;
	node 142 [expr $Pier1 + $phlat1445] $Floor4;
	node 243 [expr $Pier2 - $phlat2345] $Floor4;
	node 244 [expr $Pier2 - $phlat2345] $Floor4;
	node 241 [expr $Pier2 + $phlat2345] $Floor4;
	node 242 [expr $Pier2 + $phlat2345] $Floor4;
	node 343 [expr $Pier3 - $phlat2345] $Floor4;
	node 344 [expr $Pier3 - $phlat2345] $Floor4;
	node 341 [expr $Pier3 + $phlat2345] $Floor4;
	node 342 [expr $Pier3 + $phlat2345] $Floor4;
	node 443 [expr $Pier4 - $phlat1445] $Floor4;
	node 444 [expr $Pier4 - $phlat1445] $Floor4;
	# beam hinges at Floor 5
	node 151 [expr $Pier1 + $phlat1445] $Floor5;
	node 152 [expr $Pier1 + $phlat1445] $Floor5;
	node 253 [expr $Pier2 - $phlat2345] $Floor5;
	node 254 [expr $Pier2 - $phlat2345] $Floor5;
	node 251 [expr $Pier2 + $phlat2345] $Floor5;
	node 252 [expr $Pier2 + $phlat2345] $Floor5;
	node 353 [expr $Pier3 - $phlat2345] $Floor5;
	node 354 [expr $Pier3 - $phlat2345] $Floor5;
	node 351 [expr $Pier3 + $phlat2345] $Floor5;
	node 352 [expr $Pier3 + $phlat2345] $Floor5;
	node 453 [expr $Pier4 - $phlat1445] $Floor5;
	node 454 [expr $Pier4 - $phlat1445] $Floor5;
	# beam hinges at Floor 6
	node 161 [expr $Pier1 + $phlat1467] $Floor6;
	node 162 [expr $Pier1 + $phlat1467] $Floor6;
	node 263 [expr $Pier2 - $phlat2367] $Floor6;
	node 264 [expr $Pier2 - $phlat2367] $Floor6;
	node 261 [expr $Pier2 + $phlat2367] $Floor6;
	node 262 [expr $Pier2 + $phlat2367] $Floor6;
	node 363 [expr $Pier3 - $phlat2367] $Floor6;
	node 364 [expr $Pier3 - $phlat2367] $Floor6;
	node 361 [expr $Pier3 + $phlat2367] $Floor6;
	node 362 [expr $Pier3 + $phlat2367] $Floor6;
	node 463 [expr $Pier4 - $phlat1467] $Floor6;
	node 464 [expr $Pier4 - $phlat1467] $Floor6;
	# beam hinges at Floor 7
	node 171 [expr $Pier1 + $phlat1467] $Floor7;
	node 172 [expr $Pier1 + $phlat1467] $Floor7;
	node 273 [expr $Pier2 - $phlat2367] $Floor7;
	node 274 [expr $Pier2 - $phlat2367] $Floor7;
	node 271 [expr $Pier2 + $phlat2367] $Floor7;
	node 272 [expr $Pier2 + $phlat2367] $Floor7;
	node 373 [expr $Pier3 - $phlat2367] $Floor7;
	node 374 [expr $Pier3 - $phlat2367] $Floor7;
	node 371 [expr $Pier3 + $phlat2367] $Floor7;
	node 372 [expr $Pier3 + $phlat2367] $Floor7;
	node 473 [expr $Pier4 - $phlat1467] $Floor7;
	node 474 [expr $Pier4 - $phlat1467] $Floor7;
	# beam hinges at Floor 8
	node 181 [expr $Pier1 + $phlat1489] $Floor8;
	node 182 [expr $Pier1 + $phlat1489] $Floor8;
	node 283 [expr $Pier2 - $phlat2389] $Floor8;
	node 284 [expr $Pier2 - $phlat2389] $Floor8;
	node 281 [expr $Pier2 + $phlat2389] $Floor8;
	node 282 [expr $Pier2 + $phlat2389] $Floor8;
	node 383 [expr $Pier3 - $phlat2389] $Floor8;
	node 384 [expr $Pier3 - $phlat2389] $Floor8;
	node 381 [expr $Pier3 + $phlat2389] $Floor8;
	node 382 [expr $Pier3 + $phlat2389] $Floor8;
	node 483 [expr $Pier4 - $phlat1489] $Floor8;
	node 484 [expr $Pier4 - $phlat1489] $Floor8;
	# beam hinges at Floor 9
	node 191 [expr $Pier1 + $phlat1489] $Floor9;
	node 192 [expr $Pier1 + $phlat1489] $Floor9;
	node 293 [expr $Pier2 - $phlat2389] $Floor9;
	node 294 [expr $Pier2 - $phlat2389] $Floor9;
	node 291 [expr $Pier2 + $phlat2389] $Floor9;
	node 292 [expr $Pier2 + $phlat2389] $Floor9;
	node 393 [expr $Pier3 - $phlat2389] $Floor9;
	node 394 [expr $Pier3 - $phlat2389] $Floor9;
	node 391 [expr $Pier3 + $phlat2389] $Floor9;
	node 392 [expr $Pier3 + $phlat2389] $Floor9;
	node 493 [expr $Pier4 - $phlat1489] $Floor9;
	node 494 [expr $Pier4 - $phlat1489] $Floor9;
	
# define extra nodes for panel zones
	# nodeID convention:  "xybc" where x = Pier #, y = Floor #, bc = location relative to beam-column joint
	# "bc" conventions: 01,02 = top left of joint; 
	# 					03,04 = top right of joint;
	# 					05    = middle right of joint; (vertical middle, horizontal right)
	# 					06,07 = btm right of joint;
	# 					08,09 = btm left of joint;
	# 					10    = middle left of joint; (vertical middle, horizontal left)
	# note: top center and btm center nodes were previously defined as xy7 and xy6, respectively, at Floor 2(center = horizonal center)

	# panel zone at Pier 1, Floor 2
	node 1201 [expr $Pier1 - $pzlat1423 ] [expr $Floor2 + $phvert23];
	node 1202 [expr $Pier1 - $pzlat1423 ] [expr $Floor2 + $phvert23];
	node 1203 [expr $Pier1 + $pzlat1423 ] [expr $Floor2 + $phvert23];
	node 1204 [expr $Pier1 + $pzlat1423 ] [expr $Floor2 + $phvert23];
	node 1205 [expr $Pier1 + $pzlat1423 ] [expr $Floor2];
	node 1206 [expr $Pier1 + $pzlat1423 ] [expr $Floor2 - $phvert23];
	node 1207 [expr $Pier1 + $pzlat1423 ] [expr $Floor2 - $phvert23];
	node 1208 [expr $Pier1 - $pzlat1423 ] [expr $Floor2 - $phvert23];
	node 1209 [expr $Pier1 - $pzlat1423 ] [expr $Floor2 - $phvert23];
	node 1210 [expr $Pier1 - $pzlat1423 ] [expr $Floor2];
	
	# panel zone at Pier 2, Floor 2
	node 2201 [expr $Pier2 - $pzlat2323 ] [expr $Floor2 + $phvert23];
	node 2202 [expr $Pier2 - $pzlat2323 ] [expr $Floor2 + $phvert23];
	node 2203 [expr $Pier2 + $pzlat2323 ] [expr $Floor2 + $phvert23];
	node 2204 [expr $Pier2 + $pzlat2323 ] [expr $Floor2 + $phvert23];
	node 2205 [expr $Pier2 + $pzlat2323 ] [expr $Floor2];
	node 2206 [expr $Pier2 + $pzlat2323 ] [expr $Floor2 - $phvert23];
	node 2207 [expr $Pier2 + $pzlat2323 ] [expr $Floor2 - $phvert23];
	node 2208 [expr $Pier2 - $pzlat2323 ] [expr $Floor2 - $phvert23];
	node 2209 [expr $Pier2 - $pzlat2323 ] [expr $Floor2 - $phvert23];
	node 2210 [expr $Pier2 - $pzlat2323 ] [expr $Floor2];

	# panel zone at Pier 3, Floor 2
	node 3201 [expr $Pier3 - $pzlat2323 ] [expr $Floor2 + $phvert23];
	node 3202 [expr $Pier3 - $pzlat2323 ] [expr $Floor2 + $phvert23];
	node 3203 [expr $Pier3 + $pzlat2323 ] [expr $Floor2 + $phvert23];
	node 3204 [expr $Pier3 + $pzlat2323 ] [expr $Floor2 + $phvert23];
	node 3205 [expr $Pier3 + $pzlat2323 ] [expr $Floor2];
	node 3206 [expr $Pier3 + $pzlat2323 ] [expr $Floor2 - $phvert23];
	node 3207 [expr $Pier3 + $pzlat2323 ] [expr $Floor2 - $phvert23];
	node 3208 [expr $Pier3 - $pzlat2323 ] [expr $Floor2 - $phvert23];
	node 3209 [expr $Pier3 - $pzlat2323 ] [expr $Floor2 - $phvert23];
	node 3210 [expr $Pier3 - $pzlat2323 ] [expr $Floor2];

	# panel zone at Pier 4, Floor 2
	node 4201 [expr $Pier4 - $pzlat1423 ] [expr $Floor2 + $phvert23];
	node 4202 [expr $Pier4 - $pzlat1423 ] [expr $Floor2 + $phvert23];
	node 4203 [expr $Pier4 + $pzlat1423 ] [expr $Floor2 + $phvert23];
	node 4204 [expr $Pier4 + $pzlat1423 ] [expr $Floor2 + $phvert23];
	node 4205 [expr $Pier4 + $pzlat1423 ] [expr $Floor2];
	node 4206 [expr $Pier4 + $pzlat1423 ] [expr $Floor2 - $phvert23];
	node 4207 [expr $Pier4 + $pzlat1423 ] [expr $Floor2 - $phvert23];
	node 4208 [expr $Pier4 - $pzlat1423 ] [expr $Floor2 - $phvert23];
	node 4209 [expr $Pier4 - $pzlat1423 ] [expr $Floor2 - $phvert23];
	node 4210 [expr $Pier4 - $pzlat1423 ] [expr $Floor2];
	
	# panel zone at Pier 1, Floor 3
	node 1301 [expr $Pier1 - $pzlat1423 ] [expr $Floor3 + $phvert23];
	node 1302 [expr $Pier1 - $pzlat1423 ] [expr $Floor3 + $phvert23];
	node 1303 [expr $Pier1 + $pzlat1423 ] [expr $Floor3 + $phvert23];
	node 1304 [expr $Pier1 + $pzlat1423 ] [expr $Floor3 + $phvert23];
	node 1305 [expr $Pier1 + $pzlat1423 ] [expr $Floor3];
	node 1306 [expr $Pier1 + $pzlat1423 ] [expr $Floor3 - $phvert23];
	node 1307 [expr $Pier1 + $pzlat1423 ] [expr $Floor3 - $phvert23];
	node 1308 [expr $Pier1 - $pzlat1423 ] [expr $Floor3 - $phvert23];
	node 1309 [expr $Pier1 - $pzlat1423 ] [expr $Floor3 - $phvert23];
	node 1310 [expr $Pier1 - $pzlat1423 ] [expr $Floor3];
	
	# panel zone at Pier 2, Floor 3
	node 2301 [expr $Pier2 - $pzlat2323 ] [expr $Floor3 + $phvert23];
	node 2302 [expr $Pier2 - $pzlat2323 ] [expr $Floor3 + $phvert23];
	node 2303 [expr $Pier2 + $pzlat2323 ] [expr $Floor3 + $phvert23];
	node 2304 [expr $Pier2 + $pzlat2323 ] [expr $Floor3 + $phvert23];
	node 2305 [expr $Pier2 + $pzlat2323 ] [expr $Floor3];
	node 2306 [expr $Pier2 + $pzlat2323 ] [expr $Floor3 - $phvert23];
	node 2307 [expr $Pier2 + $pzlat2323 ] [expr $Floor3 - $phvert23];
	node 2308 [expr $Pier2 - $pzlat2323 ] [expr $Floor3 - $phvert23];
	node 2309 [expr $Pier2 - $pzlat2323 ] [expr $Floor3 - $phvert23];
	node 2310 [expr $Pier2 - $pzlat2323 ] [expr $Floor3];

	# panel zone at Pier 3, Floor 3
	node 3301 [expr $Pier3 - $pzlat2323 ] [expr $Floor3 + $phvert23];
	node 3302 [expr $Pier3 - $pzlat2323 ] [expr $Floor3 + $phvert23];
	node 3303 [expr $Pier3 + $pzlat2323 ] [expr $Floor3 + $phvert23];
	node 3304 [expr $Pier3 + $pzlat2323 ] [expr $Floor3 + $phvert23];
	node 3305 [expr $Pier3 + $pzlat2323 ] [expr $Floor3];
	node 3306 [expr $Pier3 + $pzlat2323 ] [expr $Floor3 - $phvert23];
	node 3307 [expr $Pier3 + $pzlat2323 ] [expr $Floor3 - $phvert23];
	node 3308 [expr $Pier3 - $pzlat2323 ] [expr $Floor3 - $phvert23];
	node 3309 [expr $Pier3 - $pzlat2323 ] [expr $Floor3 - $phvert23];
	node 3310 [expr $Pier3 - $pzlat2323 ] [expr $Floor3];

	# panel zone at Pier 4, Floor 3
	node 4301 [expr $Pier4 - $pzlat1423 ] [expr $Floor3 + $phvert23];
	node 4302 [expr $Pier4 - $pzlat1423 ] [expr $Floor3 + $phvert23];
	node 4303 [expr $Pier4 + $pzlat1423 ] [expr $Floor3 + $phvert23];
	node 4304 [expr $Pier4 + $pzlat1423 ] [expr $Floor3 + $phvert23];
	node 4305 [expr $Pier4 + $pzlat1423 ] [expr $Floor3];
	node 4306 [expr $Pier4 + $pzlat1423 ] [expr $Floor3 - $phvert23];
	node 4307 [expr $Pier4 + $pzlat1423 ] [expr $Floor3 - $phvert23];
	node 4308 [expr $Pier4 - $pzlat1423 ] [expr $Floor3 - $phvert23];
	node 4309 [expr $Pier4 - $pzlat1423 ] [expr $Floor3 - $phvert23];
	node 4310 [expr $Pier4 - $pzlat1423 ] [expr $Floor3];
	
	# panel zone at Pier 1, Floor 4
	node 1401 [expr $Pier1 - $pzlat1445 ] [expr $Floor4 + $phvert45];
	node 1402 [expr $Pier1 - $pzlat1445 ] [expr $Floor4 + $phvert45];
	node 1403 [expr $Pier1 + $pzlat1445 ] [expr $Floor4 + $phvert45];
	node 1404 [expr $Pier1 + $pzlat1445 ] [expr $Floor4 + $phvert45];
	node 1405 [expr $Pier1 + $pzlat1445 ] [expr $Floor4];
	node 1406 [expr $Pier1 + $pzlat1445 ] [expr $Floor4 - $phvert45];
	node 1407 [expr $Pier1 + $pzlat1445 ] [expr $Floor4 - $phvert45];
	node 1408 [expr $Pier1 - $pzlat1445 ] [expr $Floor4 - $phvert45];
	node 1409 [expr $Pier1 - $pzlat1445 ] [expr $Floor4 - $phvert45];
	node 1410 [expr $Pier1 - $pzlat1445 ] [expr $Floor4];
	
	# panel zone at Pier 2, Floor 4
	node 2401 [expr $Pier2 - $pzlat2345 ] [expr $Floor4 + $phvert45];
	node 2402 [expr $Pier2 - $pzlat2345 ] [expr $Floor4 + $phvert45];
	node 2403 [expr $Pier2 + $pzlat2345 ] [expr $Floor4 + $phvert45];
	node 2404 [expr $Pier2 + $pzlat2345 ] [expr $Floor4 + $phvert45];
	node 2405 [expr $Pier2 + $pzlat2345 ] [expr $Floor4];
	node 2406 [expr $Pier2 + $pzlat2345 ] [expr $Floor4 - $phvert45];
	node 2407 [expr $Pier2 + $pzlat2345 ] [expr $Floor4 - $phvert45];
	node 2408 [expr $Pier2 - $pzlat2345 ] [expr $Floor4 - $phvert45];
	node 2409 [expr $Pier2 - $pzlat2345 ] [expr $Floor4 - $phvert45];
	node 2410 [expr $Pier2 - $pzlat2345 ] [expr $Floor4];

	# panel zone at Pier 3, Floor 4
	node 3401 [expr $Pier3 - $pzlat2345 ] [expr $Floor4 + $phvert45];
	node 3402 [expr $Pier3 - $pzlat2345 ] [expr $Floor4 + $phvert45];
	node 3403 [expr $Pier3 + $pzlat2345 ] [expr $Floor4 + $phvert45];
	node 3404 [expr $Pier3 + $pzlat2345 ] [expr $Floor4 + $phvert45];
	node 3405 [expr $Pier3 + $pzlat2345 ] [expr $Floor4];
	node 3406 [expr $Pier3 + $pzlat2345 ] [expr $Floor4 - $phvert45];
	node 3407 [expr $Pier3 + $pzlat2345 ] [expr $Floor4 - $phvert45];
	node 3408 [expr $Pier3 - $pzlat2345 ] [expr $Floor4 - $phvert45];
	node 3409 [expr $Pier3 - $pzlat2345 ] [expr $Floor4 - $phvert45];
	node 3410 [expr $Pier3 - $pzlat2345 ] [expr $Floor4];

	# panel zone at Pier 4, Floor 4
	node 4401 [expr $Pier4 - $pzlat1445 ] [expr $Floor4 + $phvert45];
	node 4402 [expr $Pier4 - $pzlat1445 ] [expr $Floor4 + $phvert45];
	node 4403 [expr $Pier4 + $pzlat1445 ] [expr $Floor4 + $phvert45];
	node 4404 [expr $Pier4 + $pzlat1445 ] [expr $Floor4 + $phvert45];
	node 4405 [expr $Pier4 + $pzlat1445 ] [expr $Floor4];
	node 4406 [expr $Pier4 + $pzlat1445 ] [expr $Floor4 - $phvert45];
	node 4407 [expr $Pier4 + $pzlat1445 ] [expr $Floor4 - $phvert45];
	node 4408 [expr $Pier4 - $pzlat1445 ] [expr $Floor4 - $phvert45];
	node 4409 [expr $Pier4 - $pzlat1445 ] [expr $Floor4 - $phvert45];
	node 4410 [expr $Pier4 - $pzlat1445 ] [expr $Floor4];

	# panel zone at Pier 1, Floor 5
	node 1501 [expr $Pier1 - $pzlat1445 ] [expr $Floor5 + $phvert45];
	node 1502 [expr $Pier1 - $pzlat1445 ] [expr $Floor5 + $phvert45];
	node 1503 [expr $Pier1 + $pzlat1445 ] [expr $Floor5 + $phvert45];
	node 1504 [expr $Pier1 + $pzlat1445 ] [expr $Floor5 + $phvert45];
	node 1505 [expr $Pier1 + $pzlat1445 ] [expr $Floor5];
	node 1506 [expr $Pier1 + $pzlat1445 ] [expr $Floor5 - $phvert45];
	node 1507 [expr $Pier1 + $pzlat1445 ] [expr $Floor5 - $phvert45];
	node 1508 [expr $Pier1 - $pzlat1445 ] [expr $Floor5 - $phvert45];
	node 1509 [expr $Pier1 - $pzlat1445 ] [expr $Floor5 - $phvert45];
	node 1510 [expr $Pier1 - $pzlat1445 ] [expr $Floor5];
	
	# panel zone at Pier 2, Floor 5
	node 2501 [expr $Pier2 - $pzlat2345 ] [expr $Floor5 + $phvert45];
	node 2502 [expr $Pier2 - $pzlat2345 ] [expr $Floor5 + $phvert45];
	node 2503 [expr $Pier2 + $pzlat2345 ] [expr $Floor5 + $phvert45];
	node 2504 [expr $Pier2 + $pzlat2345 ] [expr $Floor5 + $phvert45];
	node 2505 [expr $Pier2 + $pzlat2345 ] [expr $Floor5];
	node 2506 [expr $Pier2 + $pzlat2345 ] [expr $Floor5 - $phvert45];
	node 2507 [expr $Pier2 + $pzlat2345 ] [expr $Floor5 - $phvert45];
	node 2508 [expr $Pier2 - $pzlat2345 ] [expr $Floor5 - $phvert45];
	node 2509 [expr $Pier2 - $pzlat2345 ] [expr $Floor5 - $phvert45];
	node 2510 [expr $Pier2 - $pzlat2345 ] [expr $Floor5];

	# panel zone at Pier 3, Floor 5
	node 3501 [expr $Pier3 - $pzlat2345 ] [expr $Floor5 + $phvert45];
	node 3502 [expr $Pier3 - $pzlat2345 ] [expr $Floor5 + $phvert45];
	node 3503 [expr $Pier3 + $pzlat2345 ] [expr $Floor5 + $phvert45];
	node 3504 [expr $Pier3 + $pzlat2345 ] [expr $Floor5 + $phvert45];
	node 3505 [expr $Pier3 + $pzlat2345 ] [expr $Floor5];
	node 3506 [expr $Pier3 + $pzlat2345 ] [expr $Floor5 - $phvert45];
	node 3507 [expr $Pier3 + $pzlat2345 ] [expr $Floor5 - $phvert45];
	node 3508 [expr $Pier3 - $pzlat2345 ] [expr $Floor5 - $phvert45];
	node 3509 [expr $Pier3 - $pzlat2345 ] [expr $Floor5 - $phvert45];
	node 3510 [expr $Pier3 - $pzlat2345 ] [expr $Floor5];

	# panel zone at Pier 4, Floor 6
	node 4501 [expr $Pier4 - $pzlat1445 ] [expr $Floor5 + $phvert45];
	node 4502 [expr $Pier4 - $pzlat1445 ] [expr $Floor5 + $phvert45];
	node 4503 [expr $Pier4 + $pzlat1445 ] [expr $Floor5 + $phvert45];
	node 4504 [expr $Pier4 + $pzlat1445 ] [expr $Floor5 + $phvert45];
	node 4505 [expr $Pier4 + $pzlat1445 ] [expr $Floor5];
	node 4506 [expr $Pier4 + $pzlat1445 ] [expr $Floor5 - $phvert45];
	node 4507 [expr $Pier4 + $pzlat1445 ] [expr $Floor5 - $phvert45];
	node 4508 [expr $Pier4 - $pzlat1445 ] [expr $Floor5 - $phvert45];
	node 4509 [expr $Pier4 - $pzlat1445 ] [expr $Floor5 - $phvert45];
	node 4510 [expr $Pier4 - $pzlat1445 ] [expr $Floor5];

	# panel zone at Pier 1, Floor 6
	node 1601 [expr $Pier1 - $pzlat1467 ] [expr $Floor6 + $phvert67];
	node 1602 [expr $Pier1 - $pzlat1467 ] [expr $Floor6 + $phvert67];
	node 1603 [expr $Pier1 + $pzlat1467 ] [expr $Floor6 + $phvert67];
	node 1604 [expr $Pier1 + $pzlat1467 ] [expr $Floor6 + $phvert67];
	node 1605 [expr $Pier1 + $pzlat1467 ] [expr $Floor6];
	node 1606 [expr $Pier1 + $pzlat1467 ] [expr $Floor6 - $phvert67];
	node 1607 [expr $Pier1 + $pzlat1467 ] [expr $Floor6 - $phvert67];
	node 1608 [expr $Pier1 - $pzlat1467 ] [expr $Floor6 - $phvert67];
	node 1609 [expr $Pier1 - $pzlat1467 ] [expr $Floor6 - $phvert67];
	node 1610 [expr $Pier1 - $pzlat1467 ] [expr $Floor6];
	
	# panel zone at Pier 2, Floor 6
	node 2601 [expr $Pier2 - $pzlat2367 ] [expr $Floor6 + $phvert67];
	node 2602 [expr $Pier2 - $pzlat2367 ] [expr $Floor6 + $phvert67];
	node 2603 [expr $Pier2 + $pzlat2367 ] [expr $Floor6 + $phvert67];
	node 2604 [expr $Pier2 + $pzlat2367 ] [expr $Floor6 + $phvert67];
	node 2605 [expr $Pier2 + $pzlat2367 ] [expr $Floor6];
	node 2606 [expr $Pier2 + $pzlat2367 ] [expr $Floor6 - $phvert67];
	node 2607 [expr $Pier2 + $pzlat2367 ] [expr $Floor6 - $phvert67];
	node 2608 [expr $Pier2 - $pzlat2367 ] [expr $Floor6 - $phvert67];
	node 2609 [expr $Pier2 - $pzlat2367 ] [expr $Floor6 - $phvert67];
	node 2610 [expr $Pier2 - $pzlat2367 ] [expr $Floor6];

	# panel zone at Pier 3, Floor 6
	node 3601 [expr $Pier3 - $pzlat2367 ] [expr $Floor6 + $phvert67];
	node 3602 [expr $Pier3 - $pzlat2367 ] [expr $Floor6 + $phvert67];
	node 3603 [expr $Pier3 + $pzlat2367 ] [expr $Floor6 + $phvert67];
	node 3604 [expr $Pier3 + $pzlat2367 ] [expr $Floor6 + $phvert67];
	node 3605 [expr $Pier3 + $pzlat2367 ] [expr $Floor6];
	node 3606 [expr $Pier3 + $pzlat2367 ] [expr $Floor6 - $phvert67];
	node 3607 [expr $Pier3 + $pzlat2367 ] [expr $Floor6 - $phvert67];
	node 3608 [expr $Pier3 - $pzlat2367 ] [expr $Floor6 - $phvert67];
	node 3609 [expr $Pier3 - $pzlat2367 ] [expr $Floor6 - $phvert67];
	node 3610 [expr $Pier3 - $pzlat2367 ] [expr $Floor6];

	# panel zone at Pier 4, Floor 6
	node 4601 [expr $Pier4 - $pzlat1467 ] [expr $Floor6 + $phvert67];
	node 4602 [expr $Pier4 - $pzlat1467 ] [expr $Floor6 + $phvert67];
	node 4603 [expr $Pier4 + $pzlat1467 ] [expr $Floor6 + $phvert67];
	node 4604 [expr $Pier4 + $pzlat1467 ] [expr $Floor6 + $phvert67];
	node 4605 [expr $Pier4 + $pzlat1467 ] [expr $Floor6];
	node 4606 [expr $Pier4 + $pzlat1467 ] [expr $Floor6 - $phvert67];
	node 4607 [expr $Pier4 + $pzlat1467 ] [expr $Floor6 - $phvert67];
	node 4608 [expr $Pier4 - $pzlat1467 ] [expr $Floor6 - $phvert67];
	node 4609 [expr $Pier4 - $pzlat1467 ] [expr $Floor6 - $phvert67];
	node 4610 [expr $Pier4 - $pzlat1467 ] [expr $Floor6];

	# panel zone at Pier 1, Floor 7
	node 1701 [expr $Pier1 - $pzlat1467 ] [expr $Floor7 + $phvert67];
	node 1702 [expr $Pier1 - $pzlat1467 ] [expr $Floor7 + $phvert67];
	node 1703 [expr $Pier1 + $pzlat1467 ] [expr $Floor7 + $phvert67];
	node 1704 [expr $Pier1 + $pzlat1467 ] [expr $Floor7 + $phvert67];
	node 1705 [expr $Pier1 + $pzlat1467 ] [expr $Floor7];
	node 1706 [expr $Pier1 + $pzlat1467 ] [expr $Floor7 - $phvert67];
	node 1707 [expr $Pier1 + $pzlat1467 ] [expr $Floor7 - $phvert67];
	node 1708 [expr $Pier1 - $pzlat1467 ] [expr $Floor7 - $phvert67];
	node 1709 [expr $Pier1 - $pzlat1467 ] [expr $Floor7 - $phvert67];
	node 1710 [expr $Pier1 - $pzlat1467 ] [expr $Floor7];
	
	# panel zone at Pier 2, Floor 7
	node 2701 [expr $Pier2 - $pzlat2367 ] [expr $Floor7 + $phvert67];
	node 2702 [expr $Pier2 - $pzlat2367 ] [expr $Floor7 + $phvert67];
	node 2703 [expr $Pier2 + $pzlat2367 ] [expr $Floor7 + $phvert67];
	node 2704 [expr $Pier2 + $pzlat2367 ] [expr $Floor7 + $phvert67];
	node 2705 [expr $Pier2 + $pzlat2367 ] [expr $Floor7];
	node 2706 [expr $Pier2 + $pzlat2367 ] [expr $Floor7 - $phvert67];
	node 2707 [expr $Pier2 + $pzlat2367 ] [expr $Floor7 - $phvert67];
	node 2708 [expr $Pier2 - $pzlat2367 ] [expr $Floor7 - $phvert67];
	node 2709 [expr $Pier2 - $pzlat2367 ] [expr $Floor7 - $phvert67];
	node 2710 [expr $Pier2 - $pzlat2367 ] [expr $Floor7];

	# panel zone at Pier 3, Floor 7
	node 3701 [expr $Pier3 - $pzlat2367 ] [expr $Floor7 + $phvert67];
	node 3702 [expr $Pier3 - $pzlat2367 ] [expr $Floor7 + $phvert67];
	node 3703 [expr $Pier3 + $pzlat2367 ] [expr $Floor7 + $phvert67];
	node 3704 [expr $Pier3 + $pzlat2367 ] [expr $Floor7 + $phvert67];
	node 3705 [expr $Pier3 + $pzlat2367 ] [expr $Floor7];
	node 3706 [expr $Pier3 + $pzlat2367 ] [expr $Floor7 - $phvert67];
	node 3707 [expr $Pier3 + $pzlat2367 ] [expr $Floor7 - $phvert67];
	node 3708 [expr $Pier3 - $pzlat2367 ] [expr $Floor7 - $phvert67];
	node 3709 [expr $Pier3 - $pzlat2367 ] [expr $Floor7 - $phvert67];
	node 3710 [expr $Pier3 - $pzlat2367 ] [expr $Floor7];

	# panel zone at Pier 4, Floor 7
	node 4701 [expr $Pier4 - $pzlat1467 ] [expr $Floor7 + $phvert67];
	node 4702 [expr $Pier4 - $pzlat1467 ] [expr $Floor7 + $phvert67];
	node 4703 [expr $Pier4 + $pzlat1467 ] [expr $Floor7 + $phvert67];
	node 4704 [expr $Pier4 + $pzlat1467 ] [expr $Floor7 + $phvert67];
	node 4705 [expr $Pier4 + $pzlat1467 ] [expr $Floor7];
	node 4706 [expr $Pier4 + $pzlat1467 ] [expr $Floor7 - $phvert67];
	node 4707 [expr $Pier4 + $pzlat1467 ] [expr $Floor7 - $phvert67];
	node 4708 [expr $Pier4 - $pzlat1467 ] [expr $Floor7 - $phvert67];
	node 4709 [expr $Pier4 - $pzlat1467 ] [expr $Floor7 - $phvert67];
	node 4710 [expr $Pier4 - $pzlat1467 ] [expr $Floor7];

	# panel zone at Pier 1, Floor 8
	node 1801 [expr $Pier1 - $pzlat1489 ] [expr $Floor8 + $phvert89];
	node 1802 [expr $Pier1 - $pzlat1489 ] [expr $Floor8 + $phvert89];
	node 1803 [expr $Pier1 + $pzlat1489 ] [expr $Floor8 + $phvert89];
	node 1804 [expr $Pier1 + $pzlat1489 ] [expr $Floor8 + $phvert89];
	node 1805 [expr $Pier1 + $pzlat1489 ] [expr $Floor8];
	node 1806 [expr $Pier1 + $pzlat1489 ] [expr $Floor8 - $phvert89];
	node 1807 [expr $Pier1 + $pzlat1489 ] [expr $Floor8 - $phvert89];
	node 1808 [expr $Pier1 - $pzlat1489 ] [expr $Floor8 - $phvert89];
	node 1809 [expr $Pier1 - $pzlat1489 ] [expr $Floor8 - $phvert89];
	node 1810 [expr $Pier1 - $pzlat1489 ] [expr $Floor8];
	
	# panel zone at Pier 2, Floor 8
	node 2801 [expr $Pier2 - $pzlat2389 ] [expr $Floor8 + $phvert89];
	node 2802 [expr $Pier2 - $pzlat2389 ] [expr $Floor8 + $phvert89];
	node 2803 [expr $Pier2 + $pzlat2389 ] [expr $Floor8 + $phvert89];
	node 2804 [expr $Pier2 + $pzlat2389 ] [expr $Floor8 + $phvert89];
	node 2805 [expr $Pier2 + $pzlat2389 ] [expr $Floor8];
	node 2806 [expr $Pier2 + $pzlat2389 ] [expr $Floor8 - $phvert89];
	node 2807 [expr $Pier2 + $pzlat2389 ] [expr $Floor8 - $phvert89];
	node 2808 [expr $Pier2 - $pzlat2389 ] [expr $Floor8 - $phvert89];
	node 2809 [expr $Pier2 - $pzlat2389 ] [expr $Floor8 - $phvert89];
	node 2810 [expr $Pier2 - $pzlat2389 ] [expr $Floor8];

	# panel zone at Pier 3, Floor 8
	node 3801 [expr $Pier3 - $pzlat2389 ] [expr $Floor8 + $phvert89];
	node 3802 [expr $Pier3 - $pzlat2389 ] [expr $Floor8 + $phvert89];
	node 3803 [expr $Pier3 + $pzlat2389 ] [expr $Floor8 + $phvert89];
	node 3804 [expr $Pier3 + $pzlat2389 ] [expr $Floor8 + $phvert89];
	node 3805 [expr $Pier3 + $pzlat2389 ] [expr $Floor8];
	node 3806 [expr $Pier3 + $pzlat2389 ] [expr $Floor8 - $phvert89];
	node 3807 [expr $Pier3 + $pzlat2389 ] [expr $Floor8 - $phvert89];
	node 3808 [expr $Pier3 - $pzlat2389 ] [expr $Floor8 - $phvert89];
	node 3809 [expr $Pier3 - $pzlat2389 ] [expr $Floor8 - $phvert89];
	node 3810 [expr $Pier3 - $pzlat2389 ] [expr $Floor8];

	# panel zone at Pier 4, Floor 8
	node 4801 [expr $Pier4 - $pzlat1489 ] [expr $Floor8 + $phvert89];
	node 4802 [expr $Pier4 - $pzlat1489 ] [expr $Floor8 + $phvert89];
	node 4803 [expr $Pier4 + $pzlat1489 ] [expr $Floor8 + $phvert89];
	node 4804 [expr $Pier4 + $pzlat1489 ] [expr $Floor8 + $phvert89];
	node 4805 [expr $Pier4 + $pzlat1489 ] [expr $Floor8];
	node 4806 [expr $Pier4 + $pzlat1489 ] [expr $Floor8 - $phvert89];
	node 4807 [expr $Pier4 + $pzlat1489 ] [expr $Floor8 - $phvert89];
	node 4808 [expr $Pier4 - $pzlat1489 ] [expr $Floor8 - $phvert89];
	node 4809 [expr $Pier4 - $pzlat1489 ] [expr $Floor8 - $phvert89];
	node 4810 [expr $Pier4 - $pzlat1489 ] [expr $Floor8];

	# panel zone at Pier 1, Floor 9
	node 1901 [expr $Pier1 - $pzlat1489 ] [expr $Floor9 + $phvert89];
	node 1902 [expr $Pier1 - $pzlat1489 ] [expr $Floor9 + $phvert89];
	node 1903 [expr $Pier1 + $pzlat1489 ] [expr $Floor9 + $phvert89];
	node 1904 [expr $Pier1 + $pzlat1489 ] [expr $Floor9 + $phvert89];
	node 1905 [expr $Pier1 + $pzlat1489 ] [expr $Floor9];
	node 1906 [expr $Pier1 + $pzlat1489 ] [expr $Floor9 - $phvert89];
	node 1907 [expr $Pier1 + $pzlat1489 ] [expr $Floor9 - $phvert89];
	node 1908 [expr $Pier1 - $pzlat1489 ] [expr $Floor9 - $phvert89];
	node 1909 [expr $Pier1 - $pzlat1489 ] [expr $Floor9 - $phvert89];
	node 1910 [expr $Pier1 - $pzlat1489 ] [expr $Floor9];
	node 197  [expr $Pier1]  [expr $Floor9 + $phvert89]; # not previously defined since no column above
	
	# panel zone at Pier 2, Floor 9
	node 2901 [expr $Pier2 - $pzlat2389 ] [expr $Floor9 + $phvert89];
	node 2902 [expr $Pier2 - $pzlat2389 ] [expr $Floor9 + $phvert89];
	node 2903 [expr $Pier2 + $pzlat2389 ] [expr $Floor9 + $phvert89];
	node 2904 [expr $Pier2 + $pzlat2389 ] [expr $Floor9 + $phvert89];
	node 2905 [expr $Pier2 + $pzlat2389 ] [expr $Floor9];
	node 2906 [expr $Pier2 + $pzlat2389 ] [expr $Floor9 - $phvert89];
	node 2907 [expr $Pier2 + $pzlat2389 ] [expr $Floor9 - $phvert89];
	node 2908 [expr $Pier2 - $pzlat2389 ] [expr $Floor9 - $phvert89];
	node 2909 [expr $Pier2 - $pzlat2389 ] [expr $Floor9 - $phvert89];
	node 2910 [expr $Pier2 - $pzlat2389 ] [expr $Floor9];
	node 297  [expr $Pier2]  [expr $Floor9 + $phvert89]; # not previously defined since no column above

	# panel zone at Pier 3, Floor 9
	node 3901 [expr $Pier3 - $pzlat2389 ] [expr $Floor9 + $phvert89];
	node 3902 [expr $Pier3 - $pzlat2389 ] [expr $Floor9 + $phvert89];
	node 3903 [expr $Pier3 + $pzlat2389 ] [expr $Floor9 + $phvert89];
	node 3904 [expr $Pier3 + $pzlat2389 ] [expr $Floor9 + $phvert89];
	node 3905 [expr $Pier3 + $pzlat2389 ] [expr $Floor9];
	node 3906 [expr $Pier3 + $pzlat2389 ] [expr $Floor9 - $phvert89];
	node 3907 [expr $Pier3 + $pzlat2389 ] [expr $Floor9 - $phvert89];
	node 3908 [expr $Pier3 - $pzlat2389 ] [expr $Floor9 - $phvert89];
	node 3909 [expr $Pier3 - $pzlat2389 ] [expr $Floor9 - $phvert89];
	node 3910 [expr $Pier3 - $pzlat2389 ] [expr $Floor9];
	node 397  [expr $Pier3]  [expr $Floor9 + $phvert89]; # not previously defined since no column above

	# panel zone at Pier 4, Floor 9
	node 4901 [expr $Pier4 - $pzlat1489 ] [expr $Floor9 + $phvert89];
	node 4902 [expr $Pier4 - $pzlat1489 ] [expr $Floor9 + $phvert89];
	node 4903 [expr $Pier4 + $pzlat1489 ] [expr $Floor9 + $phvert89];
	node 4904 [expr $Pier4 + $pzlat1489 ] [expr $Floor9 + $phvert89];
	node 4905 [expr $Pier4 + $pzlat1489 ] [expr $Floor9];
	node 4906 [expr $Pier4 + $pzlat1489 ] [expr $Floor9 - $phvert89];
	node 4907 [expr $Pier4 + $pzlat1489 ] [expr $Floor9 - $phvert89];
	node 4908 [expr $Pier4 - $pzlat1489 ] [expr $Floor9 - $phvert89];
	node 4909 [expr $Pier4 - $pzlat1489 ] [expr $Floor9 - $phvert89];
	node 4910 [expr $Pier4 - $pzlat1489 ] [expr $Floor9];
	node 497  [expr $Pier4]  [expr $Floor9 + $phvert89]; # not previously defined since no column above
	
# define nodal masses:  lump at beam-column joints in frame
	# command: mass $nodeID5$dof1mass $dof2mass $dof3mass
	mass 1205 $NodalMass $Negligible $Negligible;	# Pier 1, Floor 2
	mass 2205 $NodalMass $Negligible $Negligible;	# Pier 2, Floor 2
	mass 3205 $NodalMass $Negligible $Negligible;	# Pier 3, Floor 2
	mass 4205 $NodalMass $Negligible $Negligible;	# Pier 4, Floor 2

	mass 1305 $NodalMass $Negligible $Negligible;	# Pier 1, Floor 3
	mass 2305 $NodalMass $Negligible $Negligible;	# Pier 2, Floor 3
	mass 3305 $NodalMass $Negligible $Negligible;	# Pier 3, Floor 3
	mass 4305 $NodalMass $Negligible $Negligible;	# Pier 4, Floor 3

	mass 1405 $NodalMass $Negligible $Negligible;	# Pier 1, Floor 4
	mass 2405 $NodalMass $Negligible $Negligible;	# Pier 2, Floor 4
	mass 3405 $NodalMass $Negligible $Negligible;	# Pier 3, Floor 4
	mass 4405 $NodalMass $Negligible $Negligible;	# Pier 4, Floor 4

	mass 1505 $NodalMass $Negligible $Negligible;	# Pier 1, Floor 5
	mass 2505 $NodalMass $Negligible $Negligible;	# Pier 2, Floor 5
	mass 3505 $NodalMass $Negligible $Negligible;	# Pier 3, Floor 5
	mass 4505 $NodalMass $Negligible $Negligible;	# Pier 4, Floor 5

	mass 1605 $NodalMass $Negligible $Negligible;	# Pier 1, Floor 6
	mass 2605 $NodalMass $Negligible $Negligible;	# Pier 2, Floor 6
	mass 3605 $NodalMass $Negligible $Negligible;	# Pier 3, Floor 6
	mass 4605 $NodalMass $Negligible $Negligible;	# Pier 4, Floor 6

	mass 1705 $NodalMass $Negligible $Negligible;	# Pier 1, Floor 7
	mass 2705 $NodalMass $Negligible $Negligible;	# Pier 2, Floor 7
	mass 3705 $NodalMass $Negligible $Negligible;	# Pier 3, Floor 7
	mass 4705 $NodalMass $Negligible $Negligible;	# Pier 4, Floor 7

	mass 1805 $NodalMass $Negligible $Negligible;	# Pier 1, Floor 8
	mass 2805 $NodalMass $Negligible $Negligible;	# Pier 2, Floor 8
	mass 3805 $NodalMass $Negligible $Negligible;	# Pier 3, Floor 8
	mass 4805 $NodalMass $Negligible $Negligible;	# Pier 4, Floor 8

	mass 1905 $NodalMass $Negligible $Negligible;	# Pier 1, Floor 9
	mass 2905 $NodalMass $Negligible $Negligible;	# Pier 2, Floor 9
	mass 3905 $NodalMass $Negligible $Negligible;	# Pier 3, Floor 9
	mass 4905 $NodalMass $Negligible $Negligible;	# Pier 4, Floor 9
	

# constrain beam-column joints in a floor to have the same lateral displacement using the "equalDOF" command
	# command: equalDOF $MasterNodeID $SlaveNodeID $dof1 $dof2...
	set dof1 1;	# constrain movement in dof 1 (x-direction)
	equalDOF 1205 2205 $dof1;		# Floor 2:  Pier 1 to Pier 2
	equalDOF 1205 3205 $dof1;		# Floor 2:  Pier 1 to Pier 3
	equalDOF 1205 4205 $dof1;		# Floor 2:  Pier 1 to Pier 4
	equalDOF 1205 52 $dof1;			# Floor 2:  Pier 1 to Pier 5

	equalDOF 1305 2305 $dof1;		# Floor 3:  Pier 1 to Pier 2
	equalDOF 1305 3305 $dof1;		# Floor 3:  Pier 1 to Pier 3
	equalDOF 1305 4305 $dof1;		# Floor 3:  Pier 1 to Pier 4
	equalDOF 1305 53 $dof1;			# Floor 3:  Pier 1 to Pier 5

	equalDOF 1405 2405 $dof1;		# Floor 4:  Pier 1 to Pier 2
	equalDOF 1405 3405 $dof1;		# Floor 4:  Pier 1 to Pier 3
	equalDOF 1405 4405 $dof1;		# Floor 4:  Pier 1 to Pier 4
	equalDOF 1405 54 $dof1;			# Floor 4:  Pier 1 to Pier 5

	equalDOF 1505 2505 $dof1;		# Floor 5:  Pier 1 to Pier 2
	equalDOF 1505 3505 $dof1;		# Floor 5:  Pier 1 to Pier 3
	equalDOF 1505 4505 $dof1;		# Floor 5:  Pier 1 to Pier 4
	equalDOF 1505 55 $dof1;			# Floor 5:  Pier 1 to Pier 5

	equalDOF 1605 2605 $dof1;		# Floor 6:  Pier 1 to Pier 2
	equalDOF 1605 3605 $dof1;		# Floor 6:  Pier 1 to Pier 3
	equalDOF 1605 4605 $dof1;		# Floor 6:  Pier 1 to Pier 4
	equalDOF 1605 56 $dof1;			# Floor 6:  Pier 1 to Pier 5

	equalDOF 1705 2705 $dof1;		# Floor 7:  Pier 1 to Pier 2
	equalDOF 1705 3705 $dof1;		# Floor 7:  Pier 1 to Pier 3
	equalDOF 1705 4705 $dof1;		# Floor 7:  Pier 1 to Pier 4
	equalDOF 1705 57 $dof1;			# Floor 7:  Pier 1 to Pier 5

	equalDOF 1805 2805 $dof1;		# Floor 8:  Pier 1 to Pier 2
	equalDOF 1805 3805 $dof1;		# Floor 8:  Pier 1 to Pier 3
	equalDOF 1805 4805 $dof1;		# Floor 8:  Pier 1 to Pier 4
	equalDOF 1805 58 $dof1;			# Floor 8:  Pier 1 to Pier 5

	equalDOF 1905 2905 $dof1;		# Floor 9:  Pier 1 to Pier 2
	equalDOF 1905 3905 $dof1;		# Floor 9:  Pier 1 to Pier 3
	equalDOF 1905 4905 $dof1;		# Floor 9:  Pier 1 to Pier 4
	equalDOF 1905 59 $dof1;			# Floor 9:  Pier 1 to Pier 5


# assign boundary condidtions 
	# command:  fix nodeID dxFixity dyFixity rzFixity
	# fixity values: 1 = constrained; 0 = unconstrained
	# fix the base of the building; pin P-delta column at base
	fix 11 1 1 1;
	fix 21 1 1 1;
	fix 31 1 1 1;
	fix 41 1 1 1;
	fix 51 1 1 0;	# P-delta column is pinned

###################################################################################################
#          Define Section Properties and Elements													  
###################################################################################################
# define material properties
	set Es 29000.0;			# steel Young's modulus
	set Fy 50.0;			# steel yield strength

# define column section W24x146 for Story 1&2 for pier 1&4 

	set Acol_1412  43.0;		# cross-sectional area
	set Icol_1412  4580.0;		# moment of inertia
	set Mycol_1412 22990.0;		# yield moment at plastic hinge location (i.e., My of RBS section)
	set dcol_1412 24.74;			# depth
	set bfcol_1412 12.9;		# flange width
	set tfcol_1412 1.09;		# flange thickness
	set twcol_1412 0.65;		# web thickness

# define column section W24x162 for Story 1&2 for pier 2&3 

	set Acol_2312  47.7;		# cross-sectional area
	set Icol_2312  5170.0;		# moment of inertia
	set Mycol_2312 25740.0;		# yield moment at plastic hinge location (i.e., My of RBS section)
	set dcol_2312 25.0;			# depth
	set bfcol_2312 12.955;		# flange width
	set tfcol_2312 1.22;		# flange thickness
	set twcol_2312 0.705;		# web thickness

# define column section W24x131 for Story 3&4 for pier 1&4 
	set Acol_1434  38.5;		# cross-sectional area
	set Icol_1434  4020.0;		# moment of inertia
	set Mycol_1434 20350.0;		# yield moment at plastic hinge location (i.e., My of RBS section)
	set dcol_1434 24.48;			# depth
	set bfcol_1434 12.855;		# flange width
	set tfcol_1434 0.96;		# flange thickness
	set twcol_1434 0.605;		# web thickness

# define column section W24x162 for Story 3&4 for pier 2&3 

	set Acol_2334  47.7;		# cross-sectional area
	set Icol_2334  5170.0;		# moment of inertia
	set Mycol_2334 25740.0;		# yield moment at plastic hinge location (i.e., My of RBS section)
	set dcol_2334 25.0;			# depth
	set bfcol_2334 12.955;		# flange width
	set tfcol_2334 1.22;		# flange thickness
	set twcol_2334 0.705;		# web thickness

# define column section W24x94 for Story 5&6 for pier 1&4 
	set Acol_1456  27.7;		# cross-sectional area
	set Icol_1456  2700.0;		# moment of inertia
	set Mycol_1456 13970.0;		# yield moment at plastic hinge location (i.e., My of RBS section)
	set dcol_1456 24.31;			# depth
	set bfcol_1456 9.065;		# flange width
	set tfcol_1456 0.875;		# flange thickness
	set twcol_1456 0.515;		# web thickness


# define column section W24x131 for Story 5&6 for pier 2&3 
	set Acol_2356  38.5;		# cross-sectional area
	set Icol_2356  4020.0;		# moment of inertia
	set Mycol_2356 20350.0;		# yield moment at plastic hinge location (i.e., My of RBS section)
	set dcol_2356 24.48;			# depth
	set bfcol_2356 12.9;		# flange width
	set tfcol_2356 0.96;		# flange thickness
	set twcol_2356 0.605;		# web thickness

# define column section W24x76 for Story 7&8 for pier 1&4 

	set Acol_1478  22.4;		# cross-sectional area
	set Icol_1478  2100.0;		# moment of inertia
	set Mycol_1478 11000.0;		# yield moment at plastic hinge location (i.e., My of RBS section)
	set dcol_1478 23.92;			# depth
	set bfcol_1478 8.99;		# flange width
	set tfcol_1478 0.68;		# flange thickness
	set twcol_1478 0.44;		# web thickness

# define column section W24x76 for Story 7&8 for pier 2&3 
	set Acol_2378  22.4;		# cross-sectional area
	set Icol_2378  2100.0;		# moment of inertia
	set Mycol_2378 11000.0;		# yield moment at plastic hinge location (i.e., My of RBS section)
	set dcol_2378 23.92;			# depth
	set bfcol_2378 8.99;		# flange width
	set tfcol_2378 0.68;		# flange thickness
	set twcol_2378 0.44;		# web thickness

# define beam section W27x94 for Floor 2&3
	set Abeam_23  27.7;		# cross-sectional area (full section properties)
	set Ibeam_23  3270.0;	# moment of inertia  (full section properties)
	set Mybeam_23 15290.0;	# yield moment at plastic hinge location (i.e., My of RBS section)
	set dbeam_23 26.92;		# depth

# define beam section W24x94 for Floor 4&5
	set Abeam_45  27.7;		# cross-sectional area (full section properties)
	set Ibeam_45 2700.0;	# moment of inertia  (full section properties)
	set Mybeam_45 13970.0;	# yield moment at plastic hinge location (i.e., My of RBS section)
	set dbeam_45 24.31;		# depth

# define beam section W24x76 for Floor 6&7
	set Abeam_67  22.4;		# cross-sectional area (full section properties)
	set Ibeam_67  2100.0;	# moment of inertia  (full section properties)
	set Mybeam_67 11000.0;	# yield moment at plastic hinge location (i.e., My of RBS section)
	set dbeam_67 23.92;		# depth

# define beam section W18x50 for Floor 8&9
	set Abeam_89  14.7;		# cross-sectional area (full section properties)
	set Ibeam_89  800.0;	# moment of inertia  (full section properties)
	set Mybeam_89 5555.0;	# yield moment at plastic hinge location (i.e., My of RBS section)
	set dbeam_89 17.99;		# depth 


# determine stiffness modifications to equate the stiffness of the spring-elastic element-spring subassembly to the stiffness of the actual frame member
	# References: (1) Ibarra, L. F., and Krawinkler, H. (2005). "Global collapse of frame structures under seismic excitations," Technical Report 152,
	#             		The John A. Blume Earthquake Engineering Research Center, Department of Civil Engineering, Stanford University, Stanford, CA.
	#			  (2) Zareian, F. and Medina, R. A. (2010). �A practical method for proper modeling of structural damping in inelastic plane
	#					structural systems,� Computers & Structures, Vol. 88, 1-2, pp. 45-53.
	# calculate modified section properties to account for spring stiffness being in series with the elastic element stiffness
	set n 10.0;		# stiffness multiplier for rotational spring

	# calculate modified moment of inertia for elastic elements between plastic hinge springs
	set Icol_1412mod  [expr $Icol_1412*($n+1.0)/$n];	# modified moment of inertia for columns in Pier 1,4 Story 1,2 
	set Icol_2312mod  [expr $Icol_2312*($n+1.0)/$n];	# modified moment of inertia for columns in Pier 2,3 Story 1,2
	set Icol_1434mod  [expr $Icol_1434*($n+1.0)/$n];	# modified moment of inertia for columns in Pier 1,4 Story 3,4
	set Icol_2334mod  [expr $Icol_2334*($n+1.0)/$n];	# modified moment of inertia for columns in Pier 2,3 Story 3,4
	set Icol_1456mod  [expr $Icol_1456*($n+1.0)/$n];	# modified moment of inertia for columns in Pier 1,4 Story 5,6
	set Icol_2356mod  [expr $Icol_2356*($n+1.0)/$n];	# modified moment of inertia for columns in Pier 2,3 Story 5,6
	set Icol_1478mod  [expr $Icol_1478*($n+1.0)/$n];	# modified moment of inertia for columns in Pier 1,4 Story 7,8
	set Icol_2378mod  [expr $Icol_2378*($n+1.0)/$n];	# modified moment of inertia for columns in Pier 2,3 Story 7,8

	set Ibeam_23mod [expr $Ibeam_23*($n+1.0)/$n];	# modified moment of inertia for beams in Floor 2,3
	set Ibeam_45mod [expr $Ibeam_45*($n+1.0)/$n];	# modified moment of inertia for beams in Floor 4,5
	set Ibeam_67mod [expr $Ibeam_67*($n+1.0)/$n];	# modified moment of inertia for beams in Floor 6,7
	set Ibeam_89mod [expr $Ibeam_89*($n+1.0)/$n];	# modified moment of inertia for beams in Floor 8,9

	# calculate modified rotational stiffness for plastic hinge springs: use length between springs // Ks_col_x1x2y x1=Pier x2=pier y=story
	set Ks_col_141   [expr $n*6.0*$Es*$Icol_1412mod/($HStory1-$phvert23)];		# rotational stiffness of Story 1, Pier 1&4 column springs 
	set Ks_col_231   [expr $n*6.0*$Es*$Icol_2312mod/($HStory1-$phvert23)];		# rotational stiffness of Story 1, Pier 2&3 column springs 
	set Ks_col_142   [expr $n*6.0*$Es*$Icol_1412mod/($HStoryTyp-$phvert23-$phvert23)];	# rotational stiffness of Story 2, Pier 1&4 column springs
	set Ks_col_232   [expr $n*6.0*$Es*$Icol_2312mod/($HStoryTyp-$phvert23-$phvert23)];	# rotational stiffness of Story 2, Pier 2&3 column springs
	set Ks_col_143   [expr $n*6.0*$Es*$Icol_1434mod/($HStoryTyp-$phvert23-$phvert45)];	# rotational stiffness of Story 3, Pier 1&4 column springs
	set Ks_col_233   [expr $n*6.0*$Es*$Icol_2334mod/($HStoryTyp-$phvert23-$phvert45)];	# rotational stiffness of Story 3, Pier 2&3 column springs
	set Ks_col_144   [expr $n*6.0*$Es*$Icol_1434mod/($HStoryTyp-$phvert45-$phvert45)];	# rotational stiffness of Story 4, Pier 1&4 column springs
	set Ks_col_234   [expr $n*6.0*$Es*$Icol_2334mod/($HStoryTyp-$phvert45-$phvert45)];	# rotational stiffness of Story 4, Pier 2&3 column springs
	set Ks_col_145   [expr $n*6.0*$Es*$Icol_1456mod/($HStoryTyp-$phvert45-$phvert67)];	# rotational stiffness of Story 5, Pier 1&4 column springs
	set Ks_col_235   [expr $n*6.0*$Es*$Icol_2356mod/($HStoryTyp-$phvert45-$phvert67)];	# rotational stiffness of Story 5, Pier 2&3 column springs
	set Ks_col_146   [expr $n*6.0*$Es*$Icol_1456mod/($HStoryTyp-$phvert67-$phvert67)];	# rotational stiffness of Story 6, Pier 1&4 column springs
	set Ks_col_236   [expr $n*6.0*$Es*$Icol_2356mod/($HStoryTyp-$phvert67-$phvert67)];	# rotational stiffness of Story 6, Pier 2&3 column springs
	set Ks_col_147   [expr $n*6.0*$Es*$Icol_1478mod/($HStoryTyp-$phvert67-$phvert89)];	# rotational stiffness of Story 7, Pier 1&4 column springs
	set Ks_col_237   [expr $n*6.0*$Es*$Icol_2378mod/($HStoryTyp-$phvert67-$phvert89)];	# rotational stiffness of Story 7, Pier 2&3 column springs
	set Ks_col_148   [expr $n*6.0*$Es*$Icol_1478mod/($HStoryTyp-$phvert89-$phvert89)];	# rotational stiffness of Story 8, Pier 1&4 column springs
	set Ks_col_238   [expr $n*6.0*$Es*$Icol_2378mod/($HStoryTyp-$phvert89-$phvert89)];	# rotational stiffness of Story 8, Pier 2&3 column springs

	#Ks_beam_y1y2z y1=floor y2floor z = bay
	set Ks_beam_231 [expr $n*6.0*$Es*$Ibeam_23mod/($WBay-$phlat1423-$phlat2323)];		# rotational stiffness of Floor 2,3 & Bay 1 beam springs
	set Ks_beam_232 [expr $n*6.0*$Es*$Ibeam_23mod/($WBay-$phlat2323-$phlat2323)];		# rotational stiffness of Floor 2,3 & Bay 2 beam springs
	set Ks_beam_233 [expr $n*6.0*$Es*$Ibeam_23mod/($WBay-$phlat1423-$phlat2323)];		# rotational stiffness of Floor 2,3 & Bay 3 beam springs
	set Ks_beam_451 [expr $n*6.0*$Es*$Ibeam_45mod/($WBay-$phlat1445-$phlat2345)];		# rotational stiffness of Floor 2,3 & Bay 1 beam springs
	set Ks_beam_452 [expr $n*6.0*$Es*$Ibeam_45mod/($WBay-$phlat2345-$phlat2345)];		# rotational stiffness of Floor 2,3 & Bay 2 beam springs
	set Ks_beam_453 [expr $n*6.0*$Es*$Ibeam_45mod/($WBay-$phlat1445-$phlat2345)];		# rotational stiffness of Floor 2,3 & Bay 3 beam springs
	set Ks_beam_671 [expr $n*6.0*$Es*$Ibeam_67mod/($WBay-$phlat1467-$phlat2367)];		# rotational stiffness of Floor 2,3 & Bay 1 beam springs
	set Ks_beam_672 [expr $n*6.0*$Es*$Ibeam_67mod/($WBay-$phlat2367-$phlat2367)];		# rotational stiffness of Floor 2,3 & Bay 2 beam springs
	set Ks_beam_673 [expr $n*6.0*$Es*$Ibeam_67mod/($WBay-$phlat1467-$phlat2367)];		# rotational stiffness of Floor 2,3 & Bay 3 beam springs
	set Ks_beam_891 [expr $n*6.0*$Es*$Ibeam_89mod/($WBay-$phlat1489-$phlat2389)];		# rotational stiffness of Floor 2,3 & Bay 1 beam springs
	set Ks_beam_892 [expr $n*6.0*$Es*$Ibeam_89mod/($WBay-$phlat2389-$phlat2389)];		# rotational stiffness of Floor 2,3 & Bay 2 beam springs
	set Ks_beam_893 [expr $n*6.0*$Es*$Ibeam_89mod/($WBay-$phlat1489-$phlat2389)];		# rotational stiffness of Floor 2,3 & Bay 3 beam springs

# set up geometric transformation of elements
	set PDeltaTransf 1;
	geomTransf PDelta $PDeltaTransf; 	# PDelta transformation

# define elastic column elements using "element" command
	# command: element elasticBeamColumn $eleID $iNode $jNode $A $E $I $transfID
	# eleID convention:  "1xy" where 1 = col, x = Pier #, y = Story #
	# Columns Story 1
	element elasticBeamColumn  111  117 125 $Acol_1412 $Es $Icol_1412mod $PDeltaTransf;	# Pier 1
	element elasticBeamColumn  121  217 225 $Acol_2312 $Es $Icol_2312mod $PDeltaTransf;	# Pier 2
	element elasticBeamColumn  131  317 325 $Acol_2312 $Es $Icol_2312mod $PDeltaTransf;	# Pier 3
	element elasticBeamColumn  141  417 425 $Acol_1412 $Es $Icol_1412mod $PDeltaTransf;	# Pier 4
	# Columns Story 2
	element elasticBeamColumn  112  128 135 $Acol_1412 $Es $Icol_1412mod $PDeltaTransf;	# Pier 1
	element elasticBeamColumn  122  228 235 $Acol_2312 $Es $Icol_2312mod $PDeltaTransf;	# Pier 2
	element elasticBeamColumn  132  328 335 $Acol_2312 $Es $Icol_2312mod $PDeltaTransf;	# Pier 3
	element elasticBeamColumn  142  428 435 $Acol_1412 $Es $Icol_1412mod $PDeltaTransf;	# Pier 4
	# Columns Story 3
	element elasticBeamColumn  113  138 145 $Acol_1434 $Es $Icol_1434mod $PDeltaTransf;	# Pier 1
	element elasticBeamColumn  123  238 245 $Acol_2334 $Es $Icol_2334mod $PDeltaTransf;	# Pier 2
	element elasticBeamColumn  133  338 345 $Acol_2334 $Es $Icol_2334mod $PDeltaTransf;	# Pier 3
	element elasticBeamColumn  143  438 445 $Acol_1434 $Es $Icol_1434mod $PDeltaTransf;	# Pier 4
	# Columns Story 4
	element elasticBeamColumn  114  148 155 $Acol_1434 $Es $Icol_1434mod $PDeltaTransf;	# Pier 1
	element elasticBeamColumn  124  248 255 $Acol_2334 $Es $Icol_2334mod $PDeltaTransf;	# Pier 2
	element elasticBeamColumn  134  348 355 $Acol_2334 $Es $Icol_2334mod $PDeltaTransf;	# Pier 3
	element elasticBeamColumn  144  448 455 $Acol_1434 $Es $Icol_1434mod $PDeltaTransf;	# Pier 4
	# Columns Story 5
	element elasticBeamColumn  115  158 165 $Acol_1456 $Es $Icol_1456mod $PDeltaTransf;	# Pier 1
	element elasticBeamColumn  125  258 265 $Acol_2356 $Es $Icol_2356mod $PDeltaTransf;	# Pier 2
	element elasticBeamColumn  135  358 365 $Acol_2356 $Es $Icol_2356mod $PDeltaTransf;	# Pier 3
	element elasticBeamColumn  145  458 465 $Acol_1456 $Es $Icol_1456mod $PDeltaTransf;	# Pier 4
	# Columns Story 6
	element elasticBeamColumn  116  168 175 $Acol_1456 $Es $Icol_1456mod $PDeltaTransf;	# Pier 1
	element elasticBeamColumn  126  268 275 $Acol_2356 $Es $Icol_2356mod $PDeltaTransf;	# Pier 2
	element elasticBeamColumn  136  368 375 $Acol_2356 $Es $Icol_2356mod $PDeltaTransf;	# Pier 3
	element elasticBeamColumn  146  468 475 $Acol_1456 $Es $Icol_1456mod $PDeltaTransf;	# Pier 4
	# Columns Story 7
	element elasticBeamColumn  117  178 185 $Acol_1478 $Es $Icol_1478mod $PDeltaTransf;	# Pier 1
	element elasticBeamColumn  127  278 285 $Acol_2378 $Es $Icol_2378mod $PDeltaTransf;	# Pier 2
	element elasticBeamColumn  137  378 385 $Acol_2378 $Es $Icol_2378mod $PDeltaTransf;	# Pier 3
	element elasticBeamColumn  147  478 485 $Acol_1478 $Es $Icol_1478mod $PDeltaTransf;	# Pier 4
	# Columns Story 8
	element elasticBeamColumn  118  188 195 $Acol_1478 $Es $Icol_1478mod $PDeltaTransf;	# Pier 1
	element elasticBeamColumn  128  288 295 $Acol_2378 $Es $Icol_2378mod $PDeltaTransf;	# Pier 2
	element elasticBeamColumn  138  388 395 $Acol_2378 $Es $Icol_2378mod $PDeltaTransf;	# Pier 3
	element elasticBeamColumn  148  488 495 $Acol_1478 $Es $Icol_1478mod $PDeltaTransf;	# Pier 4
	
# define elastic beam elements
	# element between plastic hinges: eleID convention = "2xy" where 2 = beam, x = Bay #, y = Floor #
	# element between plastic hinge and panel zone: eleID convention = "2xya" where 2 = beam, x = Bay #, y = Floor #, a = loc in bay
	#	"a" convention: 1 = left end of bay; 2 = right end of bay
	# Beams Story 1 or floor 2
	element elasticBeamColumn  2121 1205 121  $Abeam_23 $Es $Ibeam_23    $PDeltaTransf;
	element elasticBeamColumn  212  122  223  $Abeam_23 $Es $Ibeam_23mod $PDeltaTransf;
	element elasticBeamColumn  2122 224  2210 $Abeam_23 $Es $Ibeam_23    $PDeltaTransf;

	element elasticBeamColumn  2221 2205 221  $Abeam_23 $Es $Ibeam_23    $PDeltaTransf;
	element elasticBeamColumn  222  222  323  $Abeam_23 $Es $Ibeam_23mod $PDeltaTransf;
	element elasticBeamColumn  2222 324  3210 $Abeam_23 $Es $Ibeam_23    $PDeltaTransf;

	element elasticBeamColumn  2321 3205 321  $Abeam_23 $Es $Ibeam_23    $PDeltaTransf;
	element elasticBeamColumn  232  322  423  $Abeam_23 $Es $Ibeam_23mod $PDeltaTransf;
	element elasticBeamColumn  2322 424  4210 $Abeam_23 $Es $Ibeam_23    $PDeltaTransf;

	# Beams Story 2 or floor 3
	element elasticBeamColumn  2131 1305 131  $Abeam_23 $Es $Ibeam_23    $PDeltaTransf;
	element elasticBeamColumn  213  132  233  $Abeam_23 $Es $Ibeam_23mod $PDeltaTransf;
	element elasticBeamColumn  2132 234  2310 $Abeam_23 $Es $Ibeam_23    $PDeltaTransf;

	element elasticBeamColumn  2231 2305 231  $Abeam_23 $Es $Ibeam_23    $PDeltaTransf;
	element elasticBeamColumn  223  232  333  $Abeam_23 $Es $Ibeam_23mod $PDeltaTransf;
	element elasticBeamColumn  2232 334  3310 $Abeam_23 $Es $Ibeam_23    $PDeltaTransf;

	element elasticBeamColumn  2331 3305 331  $Abeam_23 $Es $Ibeam_23    $PDeltaTransf;
	element elasticBeamColumn  233  332  433  $Abeam_23 $Es $Ibeam_23mod $PDeltaTransf;
	element elasticBeamColumn  2332 434  4310 $Abeam_23 $Es $Ibeam_23    $PDeltaTransf;
	
    # Beams Story 3 or floor 4
	element elasticBeamColumn  2141 1405 141  $Abeam_45 $Es $Ibeam_45    $PDeltaTransf;
	element elasticBeamColumn  214  142  243  $Abeam_45 $Es $Ibeam_45mod $PDeltaTransf;
	element elasticBeamColumn  2142 244  2410 $Abeam_45 $Es $Ibeam_45    $PDeltaTransf;

	element elasticBeamColumn  2241 2405 241  $Abeam_45 $Es $Ibeam_45    $PDeltaTransf;
	element elasticBeamColumn  224  242  343  $Abeam_45 $Es $Ibeam_45mod $PDeltaTransf;
	element elasticBeamColumn  2242 344  3410 $Abeam_45 $Es $Ibeam_45    $PDeltaTransf;

	element elasticBeamColumn  2341 3405 341  $Abeam_45 $Es $Ibeam_45    $PDeltaTransf;
	element elasticBeamColumn  234  342  443  $Abeam_45 $Es $Ibeam_45mod $PDeltaTransf;
	element elasticBeamColumn  2342 444  4410 $Abeam_45 $Es $Ibeam_45    $PDeltaTransf;

	# Beams Story 4 or floor 5
	element elasticBeamColumn  2151 1505 151  $Abeam_45 $Es $Ibeam_45    $PDeltaTransf;
	element elasticBeamColumn  215  152  253  $Abeam_45 $Es $Ibeam_45mod $PDeltaTransf;
	element elasticBeamColumn  2152 254  2510 $Abeam_45 $Es $Ibeam_45    $PDeltaTransf;

	element elasticBeamColumn  2251 2505 251  $Abeam_45 $Es $Ibeam_45    $PDeltaTransf;
	element elasticBeamColumn  225  252  353  $Abeam_45 $Es $Ibeam_45mod $PDeltaTransf;
	element elasticBeamColumn  2252 354  3510 $Abeam_45 $Es $Ibeam_45    $PDeltaTransf;

	element elasticBeamColumn  2351 3505 351  $Abeam_45 $Es $Ibeam_45    $PDeltaTransf;
	element elasticBeamColumn  235  352  453  $Abeam_45 $Es $Ibeam_45mod $PDeltaTransf;
	element elasticBeamColumn  2352 454  4510 $Abeam_45 $Es $Ibeam_45    $PDeltaTransf;

	# Beams Story 5 or floor 6
	element elasticBeamColumn  2161 1605 161  $Abeam_67 $Es $Ibeam_67    $PDeltaTransf;
	element elasticBeamColumn  216  162  263  $Abeam_67 $Es $Ibeam_67mod $PDeltaTransf;
	element elasticBeamColumn  2162 264  2610 $Abeam_67 $Es $Ibeam_67    $PDeltaTransf;

	element elasticBeamColumn  2261 2605 261  $Abeam_67 $Es $Ibeam_67    $PDeltaTransf;
	element elasticBeamColumn  226  262  363  $Abeam_67 $Es $Ibeam_67mod $PDeltaTransf;
	element elasticBeamColumn  2262 364  3610 $Abeam_67 $Es $Ibeam_67    $PDeltaTransf;

	element elasticBeamColumn  2361 3605 361  $Abeam_67 $Es $Ibeam_67    $PDeltaTransf;
	element elasticBeamColumn  236  362  463  $Abeam_67 $Es $Ibeam_67mod $PDeltaTransf;
	element elasticBeamColumn  2362 464  4610 $Abeam_67 $Es $Ibeam_67    $PDeltaTransf;

	# Beams Story 6 or floor 7
	element elasticBeamColumn  2171 1705 171  $Abeam_67 $Es $Ibeam_67    $PDeltaTransf;
	element elasticBeamColumn  217  172  273  $Abeam_67 $Es $Ibeam_67mod $PDeltaTransf;
	element elasticBeamColumn  2172 274  2710 $Abeam_67 $Es $Ibeam_67    $PDeltaTransf;

	element elasticBeamColumn  2271 2705 271  $Abeam_67 $Es $Ibeam_67    $PDeltaTransf;
	element elasticBeamColumn  227  272  373  $Abeam_67 $Es $Ibeam_67mod $PDeltaTransf;
	element elasticBeamColumn  2272 374  3710 $Abeam_67 $Es $Ibeam_67    $PDeltaTransf;

	element elasticBeamColumn  2371 3705 371  $Abeam_67 $Es $Ibeam_67    $PDeltaTransf;
	element elasticBeamColumn  237  372  473  $Abeam_67 $Es $Ibeam_67mod $PDeltaTransf;
	element elasticBeamColumn  2372 474  4710 $Abeam_67 $Es $Ibeam_67    $PDeltaTransf;

	# Beams Story 7 or floor 8
	element elasticBeamColumn  2181 1805 181  $Abeam_89 $Es $Ibeam_89    $PDeltaTransf;
	element elasticBeamColumn  218  182  283  $Abeam_89 $Es $Ibeam_89mod $PDeltaTransf;
	element elasticBeamColumn  2182 284  2810 $Abeam_89 $Es $Ibeam_89    $PDeltaTransf;

	element elasticBeamColumn  2281 2805 281  $Abeam_89 $Es $Ibeam_89    $PDeltaTransf;
	element elasticBeamColumn  228  282  383  $Abeam_89 $Es $Ibeam_89mod $PDeltaTransf;
	element elasticBeamColumn  2282 384  3810 $Abeam_89 $Es $Ibeam_89    $PDeltaTransf;

	element elasticBeamColumn  2381 3805 381  $Abeam_89 $Es $Ibeam_89    $PDeltaTransf;
	element elasticBeamColumn  238  382  483  $Abeam_89 $Es $Ibeam_89mod $PDeltaTransf;
	element elasticBeamColumn  2382 484  4810 $Abeam_89 $Es $Ibeam_89    $PDeltaTransf;

	# Beams Story 8 or floor 9
	element elasticBeamColumn  2191 1905 191  $Abeam_89 $Es $Ibeam_89    $PDeltaTransf;
	element elasticBeamColumn  219  192  293  $Abeam_89 $Es $Ibeam_89mod $PDeltaTransf;
	element elasticBeamColumn  2192 294  2910 $Abeam_89 $Es $Ibeam_89    $PDeltaTransf;

	element elasticBeamColumn  2291 2905 291  $Abeam_89 $Es $Ibeam_89    $PDeltaTransf;
	element elasticBeamColumn  229  292  393  $Abeam_89 $Es $Ibeam_89mod $PDeltaTransf;
	element elasticBeamColumn  2292 394  3910 $Abeam_89 $Es $Ibeam_89    $PDeltaTransf;

	element elasticBeamColumn  2391 3905 391  $Abeam_89 $Es $Ibeam_89    $PDeltaTransf;
	element elasticBeamColumn  239  392  493  $Abeam_89 $Es $Ibeam_89mod $PDeltaTransf;
	element elasticBeamColumn  2392 494  4910 $Abeam_89 $Es $Ibeam_89    $PDeltaTransf;
# define p-delta columns and rigid links
	set TrussMatID 600;		# define a material ID
	set Arigid 1000.0;		# define area of truss section (make much larger than A of frame elements)
	set Irigid 100000.0;	# moment of inertia for p-delta columns  (make much larger than I of frame elements)
	uniaxialMaterial Elastic $TrussMatID $Es;		# define truss material
	# rigid links
	# command: element truss $eleID $iNode $jNode $A $materialID
	# eleID convention:  6xy, 6 = truss link, x = Bay #, y = Floor #
	element truss  642 4205 52 $Arigid $TrussMatID;	# Floor 2
	element truss  643 4305 53 $Arigid $TrussMatID;	# Floor 3
	element truss  644 4405 54 $Arigid $TrussMatID;	# Floor 4
	element truss  645 4505 55 $Arigid $TrussMatID;	# Floor 5
	element truss  646 4605 56 $Arigid $TrussMatID;	# Floor 6
	element truss  647 4705 57 $Arigid $TrussMatID;	# Floor 7
	element truss  648 4805 58 $Arigid $TrussMatID;	# Floor 8
	element truss  649 4905 59 $Arigid $TrussMatID;	# Floor 9
	
	# p-delta columns
	# eleID convention:  7xy, 7 = p-delta columns, x = Pier #, y = Story #
	element elasticBeamColumn  751  51	526 $Arigid $Es $Irigid $PDeltaTransf;	# Story 1
	element elasticBeamColumn  752  527 536 $Arigid $Es $Irigid $PDeltaTransf;	# Story 2
	element elasticBeamColumn  753  537 546 $Arigid $Es $Irigid $PDeltaTransf;	# Story 3
	element elasticBeamColumn  754  547 556 $Arigid $Es $Irigid $PDeltaTransf;	# Story 4
	element elasticBeamColumn  755  557 566 $Arigid $Es $Irigid $PDeltaTransf;	# Story 5
	element elasticBeamColumn  756  567 576 $Arigid $Es $Irigid $PDeltaTransf;	# Story 6
	element elasticBeamColumn  757  577 586 $Arigid $Es $Irigid $PDeltaTransf;	# Story 7
	element elasticBeamColumn  758  587 596 $Arigid $Es $Irigid $PDeltaTransf;	# Story 8
	
	
# define elastic panel zone elements (assume rigid)
	# elemPanelZone2D creates 8 elastic elements that form a rectangular panel zone
	# references provided in elemPanelZone2D.tcl
	# note: the nodeID and eleID of the upper left corner of the PZ must be imported
	# eleID convention:  500xya, 500 = panel zone element, x = Pier #, y = Floor #
	# "a" convention: defined in elemPanelZone2D.tcl, but 1 = top left element
	set Apz 1000.0;	# area of panel zone element (make much larger than A of frame elements)
	set Ipz 1.0e5;  # moment of intertia of panel zone element (make much larger than I of frame elements)
	# elemPanelZone2D eleID  nodeR E  A_PZ I_PZ transfTag
	elemPanelZone2D   500121 1201 $Es $Apz $Ipz $PDeltaTransf;	# Pier 1, Floor 2
	elemPanelZone2D   500221 2201 $Es $Apz $Ipz $PDeltaTransf;	# Pier 2, Floor 2
	elemPanelZone2D   500321 3201 $Es $Apz $Ipz $PDeltaTransf;	# Pier 3, Floor 2
	elemPanelZone2D   500421 4201 $Es $Apz $Ipz $PDeltaTransf;	# Pier 4, Floor 2

	elemPanelZone2D   500131 1301 $Es $Apz $Ipz $PDeltaTransf;	# Pier 1, Floor 3
	elemPanelZone2D   500231 2301 $Es $Apz $Ipz $PDeltaTransf;	# Pier 2, Floor 3
	elemPanelZone2D   500331 3301 $Es $Apz $Ipz $PDeltaTransf;	# Pier 3, Floor 3
	elemPanelZone2D   500431 4301 $Es $Apz $Ipz $PDeltaTransf;	# Pier 4, Floor 3

	elemPanelZone2D   500141 1401 $Es $Apz $Ipz $PDeltaTransf;	# Pier 1, Floor 4
	elemPanelZone2D   500241 2401 $Es $Apz $Ipz $PDeltaTransf;	# Pier 2, Floor 4
	elemPanelZone2D   500341 3401 $Es $Apz $Ipz $PDeltaTransf;	# Pier 3, Floor 4
	elemPanelZone2D   500441 4401 $Es $Apz $Ipz $PDeltaTransf;	# Pier 4, Floor 4

	elemPanelZone2D   500151 1501 $Es $Apz $Ipz $PDeltaTransf;	# Pier 1, Floor 5
	elemPanelZone2D   500251 2501 $Es $Apz $Ipz $PDeltaTransf;	# Pier 2, Floor 5
	elemPanelZone2D   500351 3501 $Es $Apz $Ipz $PDeltaTransf;	# Pier 3, Floor 5
	elemPanelZone2D   500451 4501 $Es $Apz $Ipz $PDeltaTransf;	# Pier 4, Floor 5

	elemPanelZone2D   500161 1601 $Es $Apz $Ipz $PDeltaTransf;	# Pier 1, Floor 6
	elemPanelZone2D   500261 2601 $Es $Apz $Ipz $PDeltaTransf;	# Pier 2, Floor 6
	elemPanelZone2D   500361 3601 $Es $Apz $Ipz $PDeltaTransf;	# Pier 3, Floor 6
	elemPanelZone2D   500461 4601 $Es $Apz $Ipz $PDeltaTransf;	# Pier 4, Floor 6

	elemPanelZone2D   500171 1701 $Es $Apz $Ipz $PDeltaTransf;	# Pier 1, Floor 7
	elemPanelZone2D   500271 2701 $Es $Apz $Ipz $PDeltaTransf;	# Pier 2, Floor 7
	elemPanelZone2D   500371 3701 $Es $Apz $Ipz $PDeltaTransf;	# Pier 3, Floor 7
	elemPanelZone2D   500471 4701 $Es $Apz $Ipz $PDeltaTransf;	# Pier 4, Floor 7

	elemPanelZone2D   500181 1801 $Es $Apz $Ipz $PDeltaTransf;	# Pier 1, Floor 8
	elemPanelZone2D   500281 2801 $Es $Apz $Ipz $PDeltaTransf;	# Pier 2, Floor 8
	elemPanelZone2D   500381 3801 $Es $Apz $Ipz $PDeltaTransf;	# Pier 3, Floor 8
	elemPanelZone2D   500481 4801 $Es $Apz $Ipz $PDeltaTransf;	# Pier 4, Floor 8

	elemPanelZone2D   500191 1901 $Es $Apz $Ipz $PDeltaTransf;	# Pier 1, Floor 9
	elemPanelZone2D   500291 2901 $Es $Apz $Ipz $PDeltaTransf;	# Pier 2, Floor 9
	elemPanelZone2D   500391 3901 $Es $Apz $Ipz $PDeltaTransf;	# Pier 3, Floor 9
	elemPanelZone2D   500491 4901 $Es $Apz $Ipz $PDeltaTransf;	# Pier 4, Floor 9
	
# display the model with the node numbers
	DisplayModel2D NodeNumbers;
	
###################################################################################################
#          Define Rotational Springs for Plastic Hinges, Panel Zones, and Leaning Columns												  
###################################################################################################
# define rotational spring properties and create spring elements using "rotSpring2DModIKModel" procedure
	# rotSpring2DModIKModel creates a uniaxial material spring with a bilinear response based on Modified Ibarra Krawinkler Deterioration Model
	# references provided in rotSpring2DModIKModel.tcl
	# input values for Story 1 column springs
	set McMy 1.05;			# ratio of capping moment to yield moment, Mc / My
	set LS 1000.0;			# basic strength deterioration (a very large # = no cyclic deterioration)
	set LK 1000.0;			# unloading stiffness deterioration (a very large # = no cyclic deterioration)
	set LA 1000.0;			# accelerated reloading stiffness deterioration (a very large # = no cyclic deterioration)
	set LD 1000.0;			# post-capping strength deterioration (a very large # = no deterioration)
	set cS 1.0;				# exponent for basic strength deterioration (c = 1.0 for no deterioration)
	set cK 1.0;				# exponent for unloading stiffness deterioration (c = 1.0 for no deterioration)
	set cA 1.0;				# exponent for accelerated reloading stiffness deterioration (c = 1.0 for no deterioration)
	set cD 1.0;				# exponent for post-capping strength deterioration (c = 1.0 for no deterioration)
	set th_pP 0.025;		# plastic rot capacity for pos loading
	set th_pN 0.025;		# plastic rot capacity for neg loading
	set th_pcP 0.3;			# post-capping rot capacity for pos loading
	set th_pcN 0.3;			# post-capping rot capacity for neg loading
	set ResP 0.4;			# residual strength ratio for pos loading
	set ResN 0.4;			# residual strength ratio for neg loading
	set th_uP 0.4;			# ultimate rot capacity for pos loading
	set th_uN 0.4;			# ultimate rot capacity for neg loading
	set DP 1.0;				# rate of cyclic deterioration for pos loading
	set DN 1.0;				# rate of cyclic deterioration for neg loading
	set a_mem14 [expr ($n+1.0)*($Mycol_1412*($McMy-1.0)) / ($Ks_col_141*$th_pP)];	# strain hardening ratio of spring
	set b14 [expr ($a_mem14)/(1.0+$n*(1.0-$a_mem14))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	set a_mem23 [expr ($n+1.0)*($Mycol_2312*($McMy-1.0)) / ($Ks_col_231*$th_pP)];	# strain hardening ratio of spring
	set b23 [expr ($a_mem23)/(1.0+$n*(1.0-$a_mem23))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)

# define column springs
	# Spring ID: "3xya", where 3 = col spring, x = Pier #, y = Story #, a = location in story
	# "a" convention: 1 = bottom of story, 2 = top of story
	# command: rotSpring2DModIKModel	id    ndR  ndC     K   asPos  asNeg  MyPos      MyNeg      LS    LK    LA    LD   cS   cK   cA   cD  th_p+   th_p-   th_pc+   th_pc-  Res+   Res-   th_u+   th_u-    D+     D-
	# col springs @ bottom of Story 1 (at base)
	rotSpring2DModIKModel 3111 11 117 $Ks_col_141 $b14 $b14 $Mycol_1412 [expr -$Mycol_1412] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3211 21 217 $Ks_col_231 $b23 $b23 $Mycol_2312 [expr -$Mycol_2312] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3311 31 317 $Ks_col_231 $b23 $b23 $Mycol_2312 [expr -$Mycol_2312] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3411 41 417 $Ks_col_141 $b14 $b14 $Mycol_1412 [expr -$Mycol_1412] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	#col springs @ top of Story 1 (below Floor 2)
	rotSpring2DModIKModel 3112 126 125 $Ks_col_141 $b14 $b14 $Mycol_1412 [expr -$Mycol_1412] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3212 226 225 $Ks_col_231 $b23 $b23 $Mycol_2312 [expr -$Mycol_2312] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3312 326 325 $Ks_col_231 $b23 $b23 $Mycol_2312 [expr -$Mycol_2312] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3412 426 425 $Ks_col_141 $b14 $b14 $Mycol_1412 [expr -$Mycol_1412] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	# recompute strain hardening since Story 2 is not the same height as Story 1
	set a_mem14 [expr ($n+1.0)*($Mycol_1412*($McMy-1.0)) / ($Ks_col_142*$th_pP)];	# strain hardening ratio of spring
	set b14 [expr ($a_mem14)/(1.0+$n*(1.0-$a_mem14))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	set a_mem23 [expr ($n+1.0)*($Mycol_2312*($McMy-1.0)) / ($Ks_col_232*$th_pP)];	# strain hardening ratio of spring
	set b23 [expr ($a_mem23)/(1.0+$n*(1.0-$a_mem23))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	# col springs @ bottom of Story 2 (at base)
	rotSpring2DModIKModel 3121 127 128 $Ks_col_142 $b14 $b14 $Mycol_1412 [expr -$Mycol_1412] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3221 227 228 $Ks_col_232 $b23 $b23 $Mycol_2312 [expr -$Mycol_2312] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3321 327 328 $Ks_col_232 $b23 $b23 $Mycol_2312 [expr -$Mycol_2312] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3421 427 428 $Ks_col_142 $b14 $b14 $Mycol_1412 [expr -$Mycol_1412] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	#col springs @ top of Story 2 (below Floor 3)
	rotSpring2DModIKModel 3122 136 135 $Ks_col_142 $b14 $b14 $Mycol_1412 [expr -$Mycol_1412] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3222 236 235 $Ks_col_232 $b23 $b23 $Mycol_2312 [expr -$Mycol_2312] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3322 336 335 $Ks_col_232 $b23 $b23 $Mycol_2312 [expr -$Mycol_2312] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3422 436 435 $Ks_col_142 $b14 $b14 $Mycol_1412 [expr -$Mycol_1412] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	
	set a_mem14 [expr ($n+1.0)*($Mycol_1434*($McMy-1.0)) / ($Ks_col_143*$th_pP)];	# strain hardening ratio of spring
	set b14 [expr ($a_mem14)/(1.0+$n*(1.0-$a_mem14))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	set a_mem23 [expr ($n+1.0)*($Mycol_2334*($McMy-1.0)) / ($Ks_col_233*$th_pP)];	# strain hardening ratio of spring
	set b23 [expr ($a_mem23)/(1.0+$n*(1.0-$a_mem23))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	# col springs @ bottom of Story 3 (at base)
	rotSpring2DModIKModel 3131 137 138 $Ks_col_143 $b14 $b14 $Mycol_1434 [expr -$Mycol_1434] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3231 237 238 $Ks_col_233 $b23 $b23 $Mycol_2334 [expr -$Mycol_2334] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3331 337 338 $Ks_col_233 $b23 $b23 $Mycol_2334 [expr -$Mycol_2334] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3431 437 438 $Ks_col_143 $b14 $b14 $Mycol_1434 [expr -$Mycol_1434] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	#col springs @ top of Story 3 (below Floor 4)
	rotSpring2DModIKModel 3132 146 145 $Ks_col_143 $b14 $b14 $Mycol_1434 [expr -$Mycol_1434] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3232 246 245 $Ks_col_233 $b23 $b23 $Mycol_2334 [expr -$Mycol_2334] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3332 346 345 $Ks_col_233 $b23 $b23 $Mycol_2334 [expr -$Mycol_2334] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3432 446 445 $Ks_col_143 $b14 $b14 $Mycol_1434 [expr -$Mycol_1434] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	set a_mem14 [expr ($n+1.0)*($Mycol_1434*($McMy-1.0)) / ($Ks_col_144*$th_pP)];	# strain hardening ratio of spring
	set b14 [expr ($a_mem14)/(1.0+$n*(1.0-$a_mem14))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	set a_mem23 [expr ($n+1.0)*($Mycol_2334*($McMy-1.0)) / ($Ks_col_234*$th_pP)];	# strain hardening ratio of spring
	set b23 [expr ($a_mem23)/(1.0+$n*(1.0-$a_mem23))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	# col springs @ bottom of Story 4 (at base)
	rotSpring2DModIKModel 3141 147 148 $Ks_col_144 $b14 $b14 $Mycol_1434 [expr -$Mycol_1434] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3241 247 248 $Ks_col_234 $b23 $b23 $Mycol_2334 [expr -$Mycol_2334] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3341 347 348 $Ks_col_234 $b23 $b23 $Mycol_2334 [expr -$Mycol_2334] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3441 447 448 $Ks_col_144 $b14 $b14 $Mycol_1434 [expr -$Mycol_1434] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	#col springs @ top of Story 4 (below Floor 5)
	rotSpring2DModIKModel 3142 156 155 $Ks_col_144 $b14 $b14 $Mycol_1434 [expr -$Mycol_1434] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3242 256 255 $Ks_col_234 $b23 $b23 $Mycol_2334 [expr -$Mycol_2334] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3342 356 355 $Ks_col_234 $b23 $b23 $Mycol_2334 [expr -$Mycol_2334] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3442 456 455 $Ks_col_144 $b14 $b14 $Mycol_1434 [expr -$Mycol_1434] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	set a_mem14 [expr ($n+1.0)*($Mycol_1456*($McMy-1.0)) / ($Ks_col_145*$th_pP)];	# strain hardening ratio of spring
	set b14 [expr ($a_mem14)/(1.0+$n*(1.0-$a_mem14))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	set a_mem23 [expr ($n+1.0)*($Mycol_2356*($McMy-1.0)) / ($Ks_col_235*$th_pP)];	# strain hardening ratio of spring
	set b23 [expr ($a_mem23)/(1.0+$n*(1.0-$a_mem23))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	# col springs @ bottom of Story 5 (at base)
	rotSpring2DModIKModel 3151 157 158 $Ks_col_145 $b14 $b14 $Mycol_1456 [expr -$Mycol_1456] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3251 257 258 $Ks_col_235 $b23 $b23 $Mycol_2356 [expr -$Mycol_2356] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3351 357 358 $Ks_col_235 $b23 $b23 $Mycol_2356 [expr -$Mycol_2356] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3451 457 458 $Ks_col_145 $b14 $b14 $Mycol_1456 [expr -$Mycol_1456] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	#col springs @ top of Story 5 (below Floor 6)
	rotSpring2DModIKModel 3152 166 165 $Ks_col_145 $b14 $b14 $Mycol_1456 [expr -$Mycol_1456] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3252 266 265 $Ks_col_235 $b23 $b23 $Mycol_2356 [expr -$Mycol_2356] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3352 366 365 $Ks_col_235 $b23 $b23 $Mycol_2356 [expr -$Mycol_2356] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3452 466 465 $Ks_col_145 $b14 $b14 $Mycol_1456 [expr -$Mycol_1456] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	set a_mem14 [expr ($n+1.0)*($Mycol_1456*($McMy-1.0)) / ($Ks_col_146*$th_pP)];	# strain hardening ratio of spring
	set b14 [expr ($a_mem14)/(1.0+$n*(1.0-$a_mem14))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	set a_mem23 [expr ($n+1.0)*($Mycol_2356*($McMy-1.0)) / ($Ks_col_236*$th_pP)];	# strain hardening ratio of spring
	set b23 [expr ($a_mem23)/(1.0+$n*(1.0-$a_mem23))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	# col springs @ bottom of Story 6 (at base)
	rotSpring2DModIKModel 3161 167 168 $Ks_col_146 $b14 $b14 $Mycol_1456 [expr -$Mycol_1456] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3261 267 268 $Ks_col_236 $b23 $b23 $Mycol_2356 [expr -$Mycol_2356] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3361 367 368 $Ks_col_236 $b23 $b23 $Mycol_2356 [expr -$Mycol_2356] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3461 467 468 $Ks_col_146 $b14 $b14 $Mycol_1456 [expr -$Mycol_1456] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	#col springs @ top of Story 6 (below Floor 7)
	rotSpring2DModIKModel 3162 176 175 $Ks_col_146 $b14 $b14 $Mycol_1456 [expr -$Mycol_1456] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3262 276 275 $Ks_col_236 $b23 $b23 $Mycol_2356 [expr -$Mycol_2356] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3362 376 375 $Ks_col_236 $b23 $b23 $Mycol_2356 [expr -$Mycol_2356] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3462 476 475 $Ks_col_146 $b14 $b14 $Mycol_1456 [expr -$Mycol_1456] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	set a_mem14 [expr ($n+1.0)*($Mycol_1478*($McMy-1.0)) / ($Ks_col_147*$th_pP)];	# strain hardening ratio of spring
	set b14 [expr ($a_mem14)/(1.0+$n*(1.0-$a_mem14))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	set a_mem23 [expr ($n+1.0)*($Mycol_2378*($McMy-1.0)) / ($Ks_col_237*$th_pP)];	# strain hardening ratio of spring
	set b23 [expr ($a_mem23)/(1.0+$n*(1.0-$a_mem23))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	# col springs @ bottom of Story 7 (at base)
	rotSpring2DModIKModel 3171 177 178 $Ks_col_147 $b14 $b14 $Mycol_1478 [expr -$Mycol_1478] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3271 277 278 $Ks_col_237 $b23 $b23 $Mycol_2378 [expr -$Mycol_2378] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3371 377 378 $Ks_col_237 $b23 $b23 $Mycol_2378 [expr -$Mycol_2378] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3471 477 478 $Ks_col_147 $b14 $b14 $Mycol_1478 [expr -$Mycol_1478] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	#col springs @ top of Story 7 (below Floor 8)
	rotSpring2DModIKModel 3172 186 185 $Ks_col_147 $b14 $b14 $Mycol_1478 [expr -$Mycol_1478] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3272 286 285 $Ks_col_237 $b23 $b23 $Mycol_2378 [expr -$Mycol_2378] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3372 386 385 $Ks_col_237 $b23 $b23 $Mycol_2378 [expr -$Mycol_2378] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3472 486 485 $Ks_col_147 $b14 $b14 $Mycol_1478 [expr -$Mycol_1478] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	set a_mem14 [expr ($n+1.0)*($Mycol_1478*($McMy-1.0)) / ($Ks_col_148*$th_pP)];	# strain hardening ratio of spring
	set b14 [expr ($a_mem14)/(1.0+$n*(1.0-$a_mem14))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	set a_mem23 [expr ($n+1.0)*($Mycol_2378*($McMy-1.0)) / ($Ks_col_238*$th_pP)];	# strain hardening ratio of spring
	set b23 [expr ($a_mem23)/(1.0+$n*(1.0-$a_mem23))];							# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: Eqn B.5 is incorrect)
	# col springs @ bottom of Story 8 (at base)
	rotSpring2DModIKModel 3181 187 188 $Ks_col_148 $b14 $b14 $Mycol_1478 [expr -$Mycol_1478] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3281 287 288 $Ks_col_238 $b23 $b23 $Mycol_2378 [expr -$Mycol_2378] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3381 387 388 $Ks_col_238 $b23 $b23 $Mycol_2378 [expr -$Mycol_2378] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3481 487 488 $Ks_col_148 $b14 $b14 $Mycol_1478 [expr -$Mycol_1478] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	#col springs @ top of Story 8 (below Floor 9)
	rotSpring2DModIKModel 3182 196 195 $Ks_col_148 $b14 $b14 $Mycol_1478 [expr -$Mycol_1478] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3282 296 295 $Ks_col_238 $b23 $b23 $Mycol_2378 [expr -$Mycol_2378] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3382 396 395 $Ks_col_238 $b23 $b23 $Mycol_2378 [expr -$Mycol_2378] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 3482 496 495 $Ks_col_148 $b14 $b14 $Mycol_1478 [expr -$Mycol_1478] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	# create region for frame column springs
	# command: region $regionID -ele $ele_1_ID $ele_2_ID...
	region 1 -ele 3111 3211 3311 3411 3112 3212 3312 3412 3121 3221 3321 3421 3122 3222 3322 3422 3131 3231 3331 3431 3132 3232 3332 3432 3141 3241 3341 3441 3142 3242 3342 3442 3151 3251 3351 3451 3152 3252 3352 3452 3161 3261 3361 3461 3162 3262 3362 3462 3171 3271 3371 3471 3172 3272 3372 3472 3181 3281 3381 3481 3182 3282 3382 3482 ;
	
# define beam springs
	# Spring ID: "4xya", where 4 = beam spring, x = Bay #, y = Floor #, a = location in bay
	# "a" convention: 1 = left end, 2 = right end
	# redefine the rotations since they are not the same
	set th_pP 0.02;
	set th_pN 0.02;
	set th_pcP 0.16;
	set th_pcN 0.16;
	set a_mem1 [expr ($n+1.0)*($Mybeam_23*($McMy-1.0)) / ($Ks_beam_231*$th_pP)];	# strain hardening ratio of spring
	set b1 [expr ($a_mem1)/(1.0+$n*(1.0-$a_mem1))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	set a_mem2 [expr ($n+1.0)*($Mybeam_23*($McMy-1.0)) / ($Ks_beam_232*$th_pP)];	# strain hardening ratio of spring
	set b2 [expr ($a_mem2)/(1.0+$n*(1.0-$a_mem2))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	#beam springs at Floor 2
	rotSpring2DModIKModel 4121 121 122 $Ks_beam_231 $b1 $b1 $Mybeam_23 [expr -$Mybeam_23] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4122 223 224 $Ks_beam_231 $b1 $b1 $Mybeam_23 [expr -$Mybeam_23] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4221 221 222 $Ks_beam_232 $b2 $b2 $Mybeam_23 [expr -$Mybeam_23] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4222 323 324 $Ks_beam_232 $b2 $b2 $Mybeam_23 [expr -$Mybeam_23] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4321 321 322 $Ks_beam_233 $b1 $b1 $Mybeam_23 [expr -$Mybeam_23] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4322 423 424 $Ks_beam_233 $b1 $b1 $Mybeam_23 [expr -$Mybeam_23] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	
	set a_mem1 [expr ($n+1.0)*($Mybeam_23*($McMy-1.0)) / ($Ks_beam_231*$th_pP)];	# strain hardening ratio of spring
	set b1 [expr ($a_mem1)/(1.0+$n*(1.0-$a_mem1))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	set a_mem2 [expr ($n+1.0)*($Mybeam_23*($McMy-1.0)) / ($Ks_beam_232*$th_pP)];	# strain hardening ratio of spring
	set b2 [expr ($a_mem2)/(1.0+$n*(1.0-$a_mem2))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	#beam springs at Floor 3
	rotSpring2DModIKModel 4131 131 132 $Ks_beam_231 $b1 $b1 $Mybeam_23 [expr -$Mybeam_23] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4132 233 234 $Ks_beam_231 $b1 $b1 $Mybeam_23 [expr -$Mybeam_23] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4231 231 232 $Ks_beam_232 $b2 $b2 $Mybeam_23 [expr -$Mybeam_23] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4232 333 334 $Ks_beam_232 $b2 $b2 $Mybeam_23 [expr -$Mybeam_23] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4331 331 332 $Ks_beam_233 $b1 $b1 $Mybeam_23 [expr -$Mybeam_23] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4332 433 434 $Ks_beam_233 $b1 $b1 $Mybeam_23 [expr -$Mybeam_23] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	set a_mem1 [expr ($n+1.0)*($Mybeam_45*($McMy-1.0)) / ($Ks_beam_451*$th_pP)];	# strain hardening ratio of spring
	set b1 [expr ($a_mem1)/(1.0+$n*(1.0-$a_mem1))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	set a_mem2 [expr ($n+1.0)*($Mybeam_45*($McMy-1.0)) / ($Ks_beam_452*$th_pP)];	# strain hardening ratio of spring
	set b2 [expr ($a_mem2)/(1.0+$n*(1.0-$a_mem2))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	#beam springs at Floor 4
	rotSpring2DModIKModel 4141 141 142 $Ks_beam_451 $b1 $b1 $Mybeam_45 [expr -$Mybeam_45] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4142 243 244 $Ks_beam_451 $b1 $b1 $Mybeam_45 [expr -$Mybeam_45] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4241 241 242 $Ks_beam_452 $b2 $b2 $Mybeam_45 [expr -$Mybeam_45] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4242 343 344 $Ks_beam_452 $b2 $b2 $Mybeam_45 [expr -$Mybeam_45] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4341 341 342 $Ks_beam_453 $b1 $b1 $Mybeam_45 [expr -$Mybeam_45] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4342 443 444 $Ks_beam_453 $b1 $b1 $Mybeam_45 [expr -$Mybeam_45] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	set a_mem1 [expr ($n+1.0)*($Mybeam_45*($McMy-1.0)) / ($Ks_beam_451*$th_pP)];	# strain hardening ratio of spring
	set b1 [expr ($a_mem1)/(1.0+$n*(1.0-$a_mem1))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	set a_mem2 [expr ($n+1.0)*($Mybeam_45*($McMy-1.0)) / ($Ks_beam_452*$th_pP)];	# strain hardening ratio of spring
	set b2 [expr ($a_mem2)/(1.0+$n*(1.0-$a_mem2))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	#beam springs at Floor 5
	rotSpring2DModIKModel 4151 151 152 $Ks_beam_451 $b1 $b1 $Mybeam_45 [expr -$Mybeam_45] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4152 253 254 $Ks_beam_451 $b1 $b1 $Mybeam_45 [expr -$Mybeam_45] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4251 251 252 $Ks_beam_452 $b2 $b2 $Mybeam_45 [expr -$Mybeam_45] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4252 353 354 $Ks_beam_452 $b2 $b2 $Mybeam_45 [expr -$Mybeam_45] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4351 351 352 $Ks_beam_453 $b1 $b1 $Mybeam_45 [expr -$Mybeam_45] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4352 453 454 $Ks_beam_453 $b1 $b1 $Mybeam_45 [expr -$Mybeam_45] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	set a_mem1 [expr ($n+1.0)*($Mybeam_67*($McMy-1.0)) / ($Ks_beam_671*$th_pP)];	# strain hardening ratio of spring
	set b1 [expr ($a_mem1)/(1.0+$n*(1.0-$a_mem1))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	set a_mem2 [expr ($n+1.0)*($Mybeam_67*($McMy-1.0)) / ($Ks_beam_672*$th_pP)];	# strain hardening ratio of spring
	set b2 [expr ($a_mem2)/(1.0+$n*(1.0-$a_mem2))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	#beam springs at Floor 6
	rotSpring2DModIKModel 4161 161 162 $Ks_beam_671 $b1 $b1 $Mybeam_67 [expr -$Mybeam_67] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4162 263 264 $Ks_beam_671 $b1 $b1 $Mybeam_67 [expr -$Mybeam_67] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4261 261 262 $Ks_beam_672 $b2 $b2 $Mybeam_67 [expr -$Mybeam_67] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4262 363 364 $Ks_beam_672 $b2 $b2 $Mybeam_67 [expr -$Mybeam_67] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4361 361 362 $Ks_beam_673 $b1 $b1 $Mybeam_67 [expr -$Mybeam_67] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4362 463 464 $Ks_beam_673 $b1 $b1 $Mybeam_67 [expr -$Mybeam_67] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	set a_mem1 [expr ($n+1.0)*($Mybeam_67*($McMy-1.0)) / ($Ks_beam_671*$th_pP)];	# strain hardening ratio of spring
	set b1 [expr ($a_mem1)/(1.0+$n*(1.0-$a_mem1))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	set a_mem2 [expr ($n+1.0)*($Mybeam_67*($McMy-1.0)) / ($Ks_beam_672*$th_pP)];	# strain hardening ratio of spring
	set b2 [expr ($a_mem2)/(1.0+$n*(1.0-$a_mem2))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	#beam springs at Floor 7
	rotSpring2DModIKModel 4171 171 172 $Ks_beam_671 $b1 $b1 $Mybeam_67 [expr -$Mybeam_67] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4172 273 274 $Ks_beam_671 $b1 $b1 $Mybeam_67 [expr -$Mybeam_67] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4271 271 272 $Ks_beam_672 $b2 $b2 $Mybeam_67 [expr -$Mybeam_67] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4272 373 374 $Ks_beam_672 $b2 $b2 $Mybeam_67 [expr -$Mybeam_67] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4371 371 372 $Ks_beam_673 $b1 $b1 $Mybeam_67 [expr -$Mybeam_67] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4372 473 474 $Ks_beam_673 $b1 $b1 $Mybeam_67 [expr -$Mybeam_67] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	set a_mem1 [expr ($n+1.0)*($Mybeam_89*($McMy-1.0)) / ($Ks_beam_891*$th_pP)];	# strain hardening ratio of spring
	set b1 [expr ($a_mem1)/(1.0+$n*(1.0-$a_mem1))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	set a_mem2 [expr ($n+1.0)*($Mybeam_89*($McMy-1.0)) / ($Ks_beam_892*$th_pP)];	# strain hardening ratio of spring
	set b2 [expr ($a_mem2)/(1.0+$n*(1.0-$a_mem2))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	#beam springs at Floor 8
	rotSpring2DModIKModel 4181 181 182 $Ks_beam_891 $b1 $b1 $Mybeam_89 [expr -$Mybeam_89] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4182 283 284 $Ks_beam_891 $b1 $b1 $Mybeam_89 [expr -$Mybeam_89] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4281 281 282 $Ks_beam_892 $b2 $b2 $Mybeam_89 [expr -$Mybeam_89] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4282 383 384 $Ks_beam_892 $b2 $b2 $Mybeam_89 [expr -$Mybeam_89] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4381 381 382 $Ks_beam_893 $b1 $b1 $Mybeam_89 [expr -$Mybeam_89] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4382 483 484 $Ks_beam_893 $b1 $b1 $Mybeam_89 [expr -$Mybeam_89] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	set a_mem1 [expr ($n+1.0)*($Mybeam_89*($McMy-1.0)) / ($Ks_beam_891*$th_pP)];	# strain hardening ratio of spring
	set b1 [expr ($a_mem1)/(1.0+$n*(1.0-$a_mem1))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	set a_mem2 [expr ($n+1.0)*($Mybeam_89*($McMy-1.0)) / ($Ks_beam_892*$th_pP)];	# strain hardening ratio of spring
	set b2 [expr ($a_mem2)/(1.0+$n*(1.0-$a_mem2))];								# modified strain hardening ratio of spring (Ibarra & Krawinkler 2005, note: there is mistake in Eqn B.5)
	#beam springs at Floor 9
	rotSpring2DModIKModel 4191 191 192 $Ks_beam_891 $b1 $b1 $Mybeam_89 [expr -$Mybeam_89] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4192 293 294 $Ks_beam_891 $b1 $b1 $Mybeam_89 [expr -$Mybeam_89] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4291 291 292 $Ks_beam_892 $b2 $b2 $Mybeam_89 [expr -$Mybeam_89] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4292 393 394 $Ks_beam_892 $b2 $b2 $Mybeam_89 [expr -$Mybeam_89] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4391 391 392 $Ks_beam_893 $b1 $b1 $Mybeam_89 [expr -$Mybeam_89] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;
	rotSpring2DModIKModel 4392 493 494 $Ks_beam_893 $b1 $b1 $Mybeam_89 [expr -$Mybeam_89] $LS $LK $LA $LD $cS $cK $cA $cD $th_pP $th_pN $th_pcP $th_pcN $ResP $ResN $th_uP $th_uN $DP $DN;

	
	# create region for beam springs
	region 2 -ele 4121 4122 4221 4222 4321 4322 4131 4132 4231 4232 4331 4332 4141 4142 4241 4242 4341 4342 4151 4152 4251 4252 4351 4352 4161 4162 4261 4262 4361 4362 4171 4172 4271 4272 4371 4372 4181 4182 4281 4282 4381 4382 4191 4192 4291 4292 4391 4392;
	
#define panel zone springs
	# rotPanelZone2D creates a uniaxial material spring with a trilinear response based on the Krawinkler Model
	#				It also constrains the nodes in the corners of the panel zone.
	# references provided in rotPanelZone2D.tcl
	# note: the upper right corner nodes of the PZ must be imported
	source rotPanelZone2D.tcl
	set Ry 1.2; 	# expected yield strength multiplier
	set as_PZ 0.03; # strain hardening of panel zones
	# Spring ID: "4xy00" where 4 = panel zone spring, x = Pier #, y = Floor #

	#Floor 2 PZ springs
	#             ElemID  ndR  ndC  E   Fy   dc       bf_c        tf_c       tp        db       Ry   as
	rotPanelZone2D 41200 1203 1204 $Es $Fy $dcol_1412 $bfcol_1412 $tfcol_1412 $twcol_1412 $dbeam_23 $Ry $as_PZ;
	rotPanelZone2D 42200 2203 2204 $Es $Fy $dcol_2312 $bfcol_2312 $tfcol_2312 $twcol_2312 $dbeam_23 $Ry $as_PZ;
	rotPanelZone2D 43200 3203 3204 $Es $Fy $dcol_2312 $bfcol_2312 $tfcol_2312 $twcol_2312 $dbeam_23 $Ry $as_PZ;
	rotPanelZone2D 44200 4203 4204 $Es $Fy $dcol_1412 $bfcol_1412 $tfcol_1412 $twcol_1412 $dbeam_23 $Ry $as_PZ;

	#Floor 3 PZ springs
	#             ElemID  ndR  ndC  E   Fy   dc       bf_c        tf_c       tp        db       Ry   as
	rotPanelZone2D 41300 1303 1304 $Es $Fy $dcol_1412 $bfcol_1412 $tfcol_1412 $twcol_1412 $dbeam_23 $Ry $as_PZ;
	rotPanelZone2D 42300 2303 2304 $Es $Fy $dcol_2312 $bfcol_2312 $tfcol_2312 $twcol_2312 $dbeam_23 $Ry $as_PZ;
	rotPanelZone2D 43300 3303 3304 $Es $Fy $dcol_2312 $bfcol_2312 $tfcol_2312 $twcol_2312 $dbeam_23 $Ry $as_PZ;
	rotPanelZone2D 44300 4303 4304 $Es $Fy $dcol_1412 $bfcol_1412 $tfcol_1412 $twcol_1412 $dbeam_23 $Ry $as_PZ;

	#Floor 4 PZ springs
	#             ElemID  ndR  ndC  E   Fy   dc       bf_c        tf_c       tp        db       Ry   as
	rotPanelZone2D 41400 1403 1404 $Es $Fy $dcol_1434 $bfcol_1434 $tfcol_1434 $twcol_1434 $dbeam_45 $Ry $as_PZ;
	rotPanelZone2D 42400 2403 2404 $Es $Fy $dcol_2334 $bfcol_2334 $tfcol_2334 $twcol_2334 $dbeam_45 $Ry $as_PZ;
	rotPanelZone2D 43400 3403 3404 $Es $Fy $dcol_2334 $bfcol_2334 $tfcol_2334 $twcol_2334 $dbeam_45 $Ry $as_PZ;
	rotPanelZone2D 44400 4403 4404 $Es $Fy $dcol_1434 $bfcol_1434 $tfcol_1434 $twcol_1434 $dbeam_45 $Ry $as_PZ;

	#Floor 5 PZ springs
	#             ElemID  ndR  ndC  E   Fy   dc       bf_c        tf_c       tp        db       Ry   as
	rotPanelZone2D 41500 1503 1504 $Es $Fy $dcol_1434 $bfcol_1434 $tfcol_1434 $twcol_1434 $dbeam_45 $Ry $as_PZ;
	rotPanelZone2D 42500 2503 2504 $Es $Fy $dcol_2334 $bfcol_2334 $tfcol_2334 $twcol_2334 $dbeam_45 $Ry $as_PZ;
	rotPanelZone2D 43500 3503 3504 $Es $Fy $dcol_2334 $bfcol_2334 $tfcol_2334 $twcol_2334 $dbeam_45 $Ry $as_PZ;
	rotPanelZone2D 44500 4503 4504 $Es $Fy $dcol_1434 $bfcol_1434 $tfcol_1434 $twcol_1434 $dbeam_45 $Ry $as_PZ;

	#Floor 6 PZ springs
	#             ElemID  ndR  ndC  E   Fy   dc       bf_c        tf_c       tp        db       Ry   as
	rotPanelZone2D 41600 1603 1604 $Es $Fy $dcol_1456 $bfcol_1456 $tfcol_1456 $twcol_1456 $dbeam_67 $Ry $as_PZ;
	rotPanelZone2D 42600 2603 2604 $Es $Fy $dcol_2356 $bfcol_2356 $tfcol_2356 $twcol_2356 $dbeam_67 $Ry $as_PZ;
	rotPanelZone2D 43600 3603 3604 $Es $Fy $dcol_2356 $bfcol_2356 $tfcol_2356 $twcol_2356 $dbeam_67 $Ry $as_PZ;
	rotPanelZone2D 44600 4603 4604 $Es $Fy $dcol_1456 $bfcol_1456 $tfcol_1456 $twcol_1456 $dbeam_67 $Ry $as_PZ;

	#Floor 7 PZ springs
	#             ElemID  ndR  ndC  E   Fy   dc       bf_c        tf_c       tp        db       Ry   as
	rotPanelZone2D 41700 1703 1704 $Es $Fy $dcol_1456 $bfcol_1456 $tfcol_1456 $twcol_1456 $dbeam_67 $Ry $as_PZ;
	rotPanelZone2D 42700 2703 2704 $Es $Fy $dcol_2356 $bfcol_2356 $tfcol_2356 $twcol_2356 $dbeam_67 $Ry $as_PZ;
	rotPanelZone2D 43700 3703 3704 $Es $Fy $dcol_2356 $bfcol_2356 $tfcol_2356 $twcol_2356 $dbeam_67 $Ry $as_PZ;
	rotPanelZone2D 44700 4703 4704 $Es $Fy $dcol_1456 $bfcol_1456 $tfcol_1456 $twcol_1456 $dbeam_67 $Ry $as_PZ;

	#Floor 8 PZ springs
	#             ElemID  ndR  ndC  E   Fy   dc       bf_c        tf_c       tp        db       Ry   as
	rotPanelZone2D 41800 1803 1804 $Es $Fy $dcol_1478 $bfcol_1478 $tfcol_1478 $twcol_1478 $dbeam_89 $Ry $as_PZ;
	rotPanelZone2D 42800 2803 2804 $Es $Fy $dcol_2378 $bfcol_2378 $tfcol_2378 $twcol_2378 $dbeam_89 $Ry $as_PZ;
	rotPanelZone2D 43800 3803 3804 $Es $Fy $dcol_2378 $bfcol_2378 $tfcol_2378 $twcol_2378 $dbeam_89 $Ry $as_PZ;
	rotPanelZone2D 44800 4803 4804 $Es $Fy $dcol_1478 $bfcol_1478 $tfcol_1478 $twcol_1478 $dbeam_89 $Ry $as_PZ;

	#Floor 9 PZ springs
	#             ElemID  ndR  ndC  E   Fy   dc       bf_c        tf_c       tp        db       Ry   as
	rotPanelZone2D 41900 1903 1904 $Es $Fy $dcol_1478 $bfcol_1478 $tfcol_1478 $twcol_1478 $dbeam_89 $Ry $as_PZ;
	rotPanelZone2D 42900 2903 2904 $Es $Fy $dcol_2378 $bfcol_2378 $tfcol_2378 $twcol_2378 $dbeam_89 $Ry $as_PZ;
	rotPanelZone2D 43900 3903 3904 $Es $Fy $dcol_2378 $bfcol_2378 $tfcol_2378 $twcol_2378 $dbeam_89 $Ry $as_PZ;
	rotPanelZone2D 44900 4903 4904 $Es $Fy $dcol_1478 $bfcol_1478 $tfcol_1478 $twcol_1478 $dbeam_89 $Ry $as_PZ;
	
# define p-delta column spring: zero-stiffness elastic spring	
	#Spring ID: "5xya" where 5 = leaning column spring, x = Pier #, y = Story #, a = location in story
	# "a" convention: 1 = bottom of story, 2 = top of story
	# rotLeaningCol ElemID ndR ndC 
	rotLeaningCol 5512 52 526;	# top of Story 1
	rotLeaningCol 5521 52 527;	# bottom of Story 2
	rotLeaningCol 5522 53 536;	# top of Story 2
	rotLeaningCol 5531 53 537;	# bottom of Story 3
	rotLeaningCol 5532 54 546;	# top of Story 3
	rotLeaningCol 5541 54 547;	# bottom of Story 4
	rotLeaningCol 5542 55 556;	# top of Story 4
	rotLeaningCol 5551 55 557;	# bottom of Story 5
	rotLeaningCol 5552 56 566;	# top of Story 5
	rotLeaningCol 5561 56 567;	# bottom of Story 6
	rotLeaningCol 5562 57 576;	# top of Story 6
	rotLeaningCol 5571 57 577;	# bottom of Story 7
	rotLeaningCol 5572 58 586;	# top of Story 7
	rotLeaningCol 5581 58 587;	# bottom of Story 8
	rotLeaningCol 5582 59 596;	# top of Story 8
	
	# create region for P-Delta column springs
	region 3 -ele 5512 5521 5522 5531 5532 5541 5542 5551 5552 5561 5562 5571 5572 5581 5582;
	
############################################################################
#                       Eigenvalue Analysis                    			   
############################################################################
	set pi [expr 2.0*asin(1.0)];						# Definition of pi
	set nEigenI 1;										# mode i = 1
	set nEigenJ 2;										# mode j = 2
	set lambdaN [eigen [expr $nEigenJ]];				# eigenvalue analysis for nEigenJ modes
	set lambdaI [lindex $lambdaN [expr $nEigenI-1]];	# eigenvalue mode i = 1
	set lambdaJ [lindex $lambdaN [expr $nEigenJ-1]];	# eigenvalue mode j = 2
	set w1 [expr pow($lambdaI,0.5)];					# w1 (1st mode circular frequency)
	set w2 [expr pow($lambdaJ,0.5)];					# w2 (2nd mode circular frequency)
	set T1 [expr 2.0*$pi/$w1];							# 1st mode period of the structure
	set T2 [expr 2.0*$pi/$w2];							# 2nd mode period of the structure
	puts "T1 = $T1 s";									# display the first mode period in the command window
	puts "T2 = $T2 s";									# display the second mode period in the command window
	
############################################################################
#              Gravity Loads & Gravity Analysis
############################################################################
# apply gravity loads
	#command: pattern PatternType $PatternID TimeSeriesType
	pattern Plain 101 Constant {
		
		# point loads on leaning column nodes
		# command: load node Fx Fy Mz
		
		set P_PD2 [expr -519.32];	# Floor 2
		set P_PD3 [expr -519.32];	# Floor 3
		set P_PD4 [expr -519.32];	# Floor 4
		set P_PD5 [expr -519.32];	# Floor 5
		set P_PD6 [expr -519.32];	# Floor 6
		set P_PD7 [expr -519.32];	# Floor 7
		set P_PD8 [expr -519.32];	# Floor 8
		set P_PD9 [expr -519.32];	# Floor 9
		
		
		load 52 0.0 $P_PD2 0.0;		# Floor 2
		load 53 0.0 $P_PD3 0.0;		# Floor 3
		load 54 0.0 $P_PD4 0.0;		# Floor 4
		load 55 0.0 $P_PD5 0.0;		# Floor 5
		load 56 0.0 $P_PD6 0.0;		# Floor 6
		load 57 0.0 $P_PD7 0.0;		# Floor 7
		load 58 0.0 $P_PD8 0.0;		# Floor 8
		load 59 0.0 $P_PD9 0.0;		# Floor 9
		

		
		# point loads on frame column nodes
		set P_F2 [expr 0.25*(-1.0*$FloorWeight-$P_PD2)];	# load on each frame node in Floor 2
		set P_F3 [expr 0.25*(-1.0*$FloorWeight-$P_PD3)];	# load on each frame node in Floor 3
		set P_F4 [expr 0.25*(-1.0*$FloorWeight-$P_PD4)];	# load on each frame node in Floor 4
		set P_F5 [expr 0.25*(-1.0*$FloorWeight-$P_PD5)];	# load on each frame node in Floor 5
		set P_F6 [expr 0.25*(-1.0*$FloorWeight-$P_PD6)];	# load on each frame node in Floor 6
		set P_F7 [expr 0.25*(-1.0*$FloorWeight-$P_PD7)];	# load on each frame node in Floor 7
		set P_F8 [expr 0.25*(-1.0*$FloorWeight-$P_PD8)];	# load on each frame node in Floor 8
		set P_F9 [expr 0.25*(-1.0*$FloorWeight-$P_PD9)];	# load on each frame node in Floor 9
		
		
		# Floor 2 loads
		load 127 0.0 $P_F2 0.0;
		load 227 0.0 $P_F2 0.0;
		load 327 0.0 $P_F2 0.0;
		load 427 0.0 $P_F2 0.0;	

		# Floor 3 loads
		load 137 0.0 $P_F3 0.0;
		load 237 0.0 $P_F3 0.0;
		load 337 0.0 $P_F3 0.0;
		load 437 0.0 $P_F3 0.0;

		# Floor 4 loads
		load 147 0.0 $P_F4 0.0;
		load 247 0.0 $P_F4 0.0;
		load 347 0.0 $P_F4 0.0;
		load 447 0.0 $P_F4 0.0;	

		# Floor 5 loads
		load 157 0.0 $P_F5 0.0;
		load 257 0.0 $P_F5 0.0;
		load 357 0.0 $P_F5 0.0;
		load 457 0.0 $P_F5 0.0;	

		# Floor 6 loads
		load 167 0.0 $P_F6 0.0;
		load 267 0.0 $P_F6 0.0;
		load 367 0.0 $P_F6 0.0;
		load 467 0.0 $P_F6 0.0;	

		# Floor 7 loads
		load 177 0.0 $P_F7 0.0;
		load 277 0.0 $P_F7 0.0;
		load 377 0.0 $P_F7 0.0;
		load 477 0.0 $P_F7 0.0;	

		# Floor 8 loads
		load 187 0.0 $P_F8 0.0;
		load 287 0.0 $P_F8 0.0;
		load 387 0.0 $P_F8 0.0;
		load 487 0.0 $P_F8 0.0;	

		# Floor 9 loads
		load 197 0.0 $P_F9 0.0;
		load 297 0.0 $P_F9 0.0;
		load 397 0.0 $P_F9 0.0;
		load 497 0.0 $P_F9 0.0;	
	}

# Gravity-analysis: load-controlled static analysis
	set Tol 1.0e-6;							# convergence tolerance for test
	constraints Plain;						# how it handles boundary conditions
	numberer RCM;							# renumber dof's to minimize band-width (optimization)
	system BandGeneral;						# how to store and solve the system of equations in the analysis (large model: try UmfPack)
	test NormDispIncr $Tol 6;				# determine if convergence has been achieved at the end of an iteration step
	algorithm Newton;						# use Newton's solution algorithm: updates tangent stiffness at every iteration
	set NstepGravity 10;					# apply gravity in 10 steps
	set DGravity [expr 1.0/$NstepGravity];	# load increment
	integrator LoadControl $DGravity;		# determine the next time step for an analysis
	analysis Static;						# define type of analysis: static or transient
	analyze $NstepGravity;					# apply gravity

	# maintain constant gravity loads and reset time to zero
	loadConst -time 0.0
	puts "Model Built"
	
############################################################################
#              Recorders					                			   
############################################################################
# record drift histories
	# record drifts
	recorder Drift -file $dataDir/Drift-Story1.out -time -iNode 11 -jNode 1205 -dof 1 -perpDirn 2;
	recorder Drift -file $dataDir/Drift-Story2.out -time -iNode 1205 -jNode 1305 -dof 1 -perpDirn 2;
	recorder Drift -file $dataDir/Drift-Story3.out -time -iNode 1305 -jNode 1405 -dof 1 -perpDirn 2; #need to check??????
	recorder Drift -file $dataDir/Drift-Story4.out -time -iNode 1405 -jNode 1505 -dof 1 -perpDirn 2;
	recorder Drift -file $dataDir/Drift-Story5.out -time -iNode 1505 -jNode 1605 -dof 1 -perpDirn 2;
	recorder Drift -file $dataDir/Drift-Story6.out -time -iNode 1605 -jNode 1705 -dof 1 -perpDirn 2;
	recorder Drift -file $dataDir/Drift-Story7.out -time -iNode 1705 -jNode 1805 -dof 1 -perpDirn 2;
	recorder Drift -file $dataDir/Drift-Story8.out -time -iNode 1805 -jNode 1905 -dof 1 -perpDirn 2;
	recorder Drift -file $dataDir/Drift-Roof.out -time -iNode 11 -jNode 1905 -dof 1 -perpDirn 2; #changed from 1305
	
# record floor displacements	
	recorder Node -file $dataDir/Disp.out -time -node 1205 1905 -dof 1 disp; #changed from 1305
	
# record base shear reactions
	recorder Node -file $dataDir/Vbase.out -time -node 117 417 51 -dof 1 reaction;
	
# record story 1 column forces in global coordinates 
	recorder Element -file $dataDir/Fcol111.out -time -ele 111 force;
	recorder Element -file $dataDir/Fcol121.out -time -ele 121 force;
	recorder Element -file $dataDir/Fcol131.out -time -ele 131 force;
	recorder Element -file $dataDir/Fcol141.out -time -ele 141 force;
	recorder Element -file $dataDir/Fcol751.out -time -ele 751 force;
	
# record response history of all frame column springs (one file for moment, one for rotation)
	recorder Element -file $dataDir/MRFcol-Mom-Hist.out -time -region 1 force;
	recorder Element -file $dataDir/MRFcol-Rot-Hist.out -time -region 1 deformation;
	
# record response history of all frame beam springs (one file for moment, one for rotation)
	recorder Element -file $dataDir/MRFbeam-Mom-Hist.out -time -region 2 force;
	recorder Element -file $dataDir/MRFbeam-Rot-Hist.out -time -region 2 deformation;
	
	
#######################################################################################
#                                                                                     #
#                              Analysis Section			                              #
#                                                                                     #
#######################################################################################

############################################################################
#              Pushover Analysis                			   			   #
############################################################################
if {$analysisType == "pushover"} { 
	puts "Running Pushover..."
# assign lateral loads and create load pattern:  use ASCE 7-10 distribution 
	set lat2 3.72;	# force on each beam-column joint in Floor 2
	set lat3 8.599;	# force on each beam-column joint in Floor 3
	set lat4 14.33;	# force on each beam-column joint in Floor 4
	set lat5 20.73;	# force on each beam-column joint in Floor 5
	set lat6 27.67;	# force on each beam-column joint in Floor 6
	set lat7 35.09;	# force on each beam-column joint in Floor 7
	set lat8 42.92;	# force on each beam-column joint in Floor 8
	set lat9 51.14;	# force on each beam-column joint in Floor 9
					
	pattern Plain 200 Linear {			
					load 1205 $lat2 0.0 0.0;
					load 2205 $lat2 0.0 0.0;
					load 3205 $lat2 0.0 0.0;
					load 4205 $lat2 0.0 0.0;
					
					load 1305 $lat3 0.0 0.0;
					load 2305 $lat3 0.0 0.0;
					load 3305 $lat3 0.0 0.0;
					load 4305 $lat3 0.0 0.0;

					load 1405 $lat4 0.0 0.0;
					load 2405 $lat4 0.0 0.0;
					load 3405 $lat4 0.0 0.0;
					load 4405 $lat4 0.0 0.0;

					load 1505 $lat5 0.0 0.0;
					load 2505 $lat5 0.0 0.0;
					load 3505 $lat5 0.0 0.0;
					load 4505 $lat5 0.0 0.0;

					load 1605 $lat6 0.0 0.0;
					load 2605 $lat6 0.0 0.0;
					load 3605 $lat6 0.0 0.0;
					load 4605 $lat6 0.0 0.0;

					load 1705 $lat7 0.0 0.0;
					load 2705 $lat7 0.0 0.0;
					load 3705 $lat7 0.0 0.0;
					load 4705 $lat7 0.0 0.0;

					load 1805 $lat8 0.0 0.0;
					load 2805 $lat8 0.0 0.0;
					load 3805 $lat8 0.0 0.0;
					load 4805 $lat8 0.0 0.0;

					load 1905 $lat9 0.0 0.0;
					load 2905 $lat9 0.0 0.0;
					load 3905 $lat9 0.0 0.0;
					load 4905 $lat9 0.0 0.0;
	}
	
# display deformed shape:
	set ViewScale 5;
	DisplayModel2D DeformedShape $ViewScale ;	# display deformed shape, the scaling factor needs to be adjusted for each model

# displacement parameters
	set IDctrlNode 1905;				# node where disp is read for disp control ///////// changed to 1305 to 1405
	set IDctrlDOF 1;					# degree of freedom read for disp control (1 = x displacement)
	set Dmax [expr 0.1*$HBuilding];		# maximum displacement of pushover: 10% roof drift
	set Dincr [expr 0.01];				# displacement increment

# analysis commands
	constraints Plain;					# how it handles boundary conditions
	numberer RCM;						# renumber dof's to minimize band-width (optimization)
	system BandGeneral;					# how to store and solve the system of equations in the analysis (large model: try UmfPack)
	test NormUnbalance 1.0e-5 400;		# type of convergence criteria with tolerance, max iterations
	algorithm Newton;					# use Newton's solution algorithm: updates tangent stiffness at every iteration
	integrator DisplacementControl  $IDctrlNode   $IDctrlDOF $Dincr;	# use displacement-controlled analysis
	analysis Static;					# define type of analysis: static for pushover
	set Nsteps [expr int($Dmax/$Dincr)];# number of pushover analysis steps
	set ok [analyze $Nsteps];			# this will return zero if no convergence problems were encountered
	puts "Pushover complete";			# display this message in the command window
} 	
	
############################################################################
#   Time History/Dynamic Analysis               			   			   #
############################################################################	
if {$analysisType == "dynamic"} { 
	puts "Running dynamic analysis..."
		# display deformed shape:
		set ViewScale 5;	# amplify display of deformed shape
		DisplayModel2D DeformedShape $ViewScale;	# display deformed shape, the scaling factor needs to be adjusted for each model
	
	# Rayleigh Damping
		# calculate damping parameters
		set zeta 0.02;		# percentage of critical damping
		set a0 [expr $zeta*2.0*$w1*$w2/($w1 + $w2)];	# mass damping coefficient based on first and second modes
		set a1 [expr $zeta*2.0/($w1 + $w2)];			# stiffness damping coefficient based on first and second modes
		set a1_mod [expr $a1*(1.0+$n)/$n];				# modified stiffness damping coefficient used for n modified elements. See Zareian & Medina 2010.
		
		# assign damping to frame beams and columns		
		# command: region $regionID -eleRange $elementIDfirst $elementIDlast rayleigh $alpha_mass $alpha_currentStiff $alpha_initialStiff $alpha_committedStiff
		region 4 -eleRange 111 239 rayleigh 0.0 0.0 $a1_mod 0.0;	# assign stiffness proportional damping to frame beams & columns w/ n modifications
		region 5 -eleRange 2121 2392 rayleigh 0.0 0.0 $a1 0.0;		# assign stiffness proportional damping to frame beams & columns w/out n modifications
		#region 6 -eleRange 500000 599999 rayleigh 0.0 0.0 $a1 0.0;	# assign stiffness proportional damping to panel zone elements
		rayleigh $a0 0.0 0.0 0.0;              						# assign mass proportional damping to structure (only assigns to nodes with mass)
		
	# define ground motion parameters
		set patternID 1;				# load pattern ID
		set GMdirection 1;				# ground motion direction (1 = x)
		set GMfile "NR94cnp.tcl";		# ground motion filename
		set dt 0.01;					# timestep of input GM file
		set Scalefact 1.0;				# ground motion scaling factor
		set TotalNumberOfSteps 2495;	# number of steps in ground motion
		set GMtime [expr $dt*$TotalNumberOfSteps + 10.0];	# total time of ground motion + 10 sec of free vibration
		
	# define the acceleration series for the ground motion
		# syntax:  "Series -dt $timestep_of_record -filePath $filename_with_acc_history -factor $scale_record_by_this_amount
		set accelSeries "Series -dt $dt -filePath $GMfile -factor [expr $Scalefact*$g]";
		
	# create load pattern: apply acceleration to all fixed nodes with UniformExcitation
		# command: pattern UniformExcitation $patternID $GMdir -accel $timeSeriesID 
		pattern UniformExcitation $patternID $GMdirection -accel $accelSeries;
		
	# define dynamic analysis parameters
		set dt_analysis 0.001;			# timestep of analysis
		wipeAnalysis;					# destroy all components of the Analysis object, i.e. any objects created with system, numberer, constraints, integrator, algorithm, and analysis commands
		constraints Plain;				# how it handles boundary conditions
		numberer RCM;					# renumber dof's to minimize band-width (optimization)
		system UmfPack;					# how to store and solve the system of equations in the analysis
		test NormDispIncr 1.0e-8 10;	# type of convergence criteria with tolerance, max iterations
		algorithm Newton;				# use Newton's solution algorithm: updates tangent stiffness at every iteration
		integrator Newmark 0.5 0.25;	# uses Newmark's average acceleration method to compute the time history
		analysis Transient;				# type of analysis: transient or static
		set NumSteps [expr round(($GMtime + 0.0)/$dt_analysis)];	# number of steps in analysis
		
	# perform the dynamic analysis and display whether analysis was successful	
		set ok [analyze $NumSteps $dt_analysis];	# ok = 0 if analysis was completed
		if {$ok == 0} {
			puts "Dynamic analysis complete";
		} else {
			puts "Dynamic analysis did not converge";
		}		
		
	# output time at end of analysis	
		set currentTime [getTime];	# get current analysis time	(after dynamic analysis)
		puts "The current time is: $currentTime";
		wipe all;
}
	
wipe all;