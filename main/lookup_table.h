#ifndef LOOKUP_TABLE_H
#define LOOKUP_TABLE_H

const uint16_t sine_lookup_table[] = {
32768, 32882, 32996, 33111, 33225, 33339, 33454, 33568, 33682, 33797,
33911, 34025, 34140, 34254, 34368, 34482, 34597, 34711, 34825, 34939,
35053, 35167, 35281, 35395, 35509, 35623, 35737, 35851, 35965, 36079,
36193, 36306, 36420, 36534, 36647, 36761, 36874, 36988, 37101, 37215,
37328, 37441, 37554, 37667, 37780, 37893, 38006, 38119, 38232, 38345,
38458, 38570, 38683, 38795, 38908, 39020, 39132, 39244, 39356, 39468,
39580, 39692, 39804, 39916, 40027, 40139, 40250, 40361, 40473, 40584,
40695, 40806, 40916, 41027, 41138, 41248, 41359, 41469, 41579, 41689,
41799, 41909, 42019, 42129, 42238, 42348, 42457, 42566, 42675, 42784,
42893, 43002, 43111, 43219, 43327, 43436, 43544, 43652, 43759, 43867,
43975, 44082, 44189, 44296, 44403, 44510, 44617, 44724, 44830, 44936,
45042, 45148, 45254, 45360, 45465, 45571, 45676, 45781, 45886, 45991,
46095, 46200, 46304, 46408, 46512, 46616, 46719, 46823, 46926, 47029,
47132, 47235, 47337, 47439, 47542, 47644, 47745, 47847, 47949, 48050,
48151, 48252, 48353, 48453, 48553, 48653, 48753, 48853, 48953, 49052,
49151, 49250, 49349, 49448, 49546, 49644, 49742, 49840, 49937, 50035,
50132, 50229, 50325, 50422, 50518, 50614, 50710, 50805, 50901, 50996,
51091, 51186, 51280, 51374, 51468, 51562, 51656, 51749, 51842, 51935,
52028, 52120, 52212, 52304, 52396, 52487, 52579, 52670, 52760, 52851,
52941, 53031, 53121, 53210, 53300, 53389, 53478, 53566, 53654, 53742,
53830, 53918, 54005, 54092, 54178, 54265, 54351, 54437, 54523, 54608,
54693, 54778, 54863, 54947, 55031, 55115, 55198, 55282, 55365, 55447,
55530, 55612, 55694, 55775, 55857, 55938, 56018, 56099, 56179, 56259,
56338, 56418, 56497, 56576, 56654, 56732, 56810, 56888, 56965, 57042,
57118, 57195, 57271, 57347, 57422, 57497, 57572, 57647, 57721, 57795,
57869, 57942, 58015, 58088, 58160, 58233, 58304, 58376, 58447, 58518,
58589, 58659, 58729, 58798, 58868, 58937, 59005, 59074, 59142, 59210,
59277, 59344, 59411, 59477, 59543, 59609, 59675, 59740, 59804, 59869,
59933, 59997, 60060, 60123, 60186, 60249, 60311, 60373, 60434, 60495,
60556, 60616, 60676, 60736, 60796, 60855, 60913, 60972, 61030, 61088,
61145, 61202, 61259, 61315, 61371, 61427, 61482, 61537, 61591, 61646,
61699, 61753, 61806, 61859, 61911, 61964, 62015, 62067, 62118, 62168,
62219, 62269, 62318, 62368, 62416, 62465, 62513, 62561, 62608, 62655,
62702, 62748, 62794, 62840, 62885, 62930, 62975, 63019, 63063, 63106,
63149, 63192, 63234, 63276, 63317, 63359, 63399, 63440, 63480, 63520,
63559, 63598, 63636, 63675, 63712, 63750, 63787, 63824, 63860, 63896,
63931, 63966, 64001, 64036, 64070, 64103, 64136, 64169, 64202, 64234,
64266, 64297, 64328, 64358, 64389, 64418, 64448, 64477, 64506, 64534,
64562, 64589, 64616, 64643, 64669, 64695, 64721, 64746, 64771, 64795,
64819, 64843, 64866, 64889, 64911, 64933, 64955, 64976, 64997, 65017,
65037, 65057, 65076, 65095, 65113, 65132, 65149, 65167, 65183, 65200,
65216, 65232, 65247, 65262, 65277, 65291, 65304, 65318, 65331, 65343,
65355, 65367, 65379, 65390, 65400, 65410, 65420, 65429, 65438, 65447,
65455, 65463, 65470, 65477, 65484, 65490, 65496, 65501, 65506, 65511,
65515, 65519, 65522, 65525, 65528, 65530, 65532, 65533, 65534, 65535,
65535, 65535, 65534, 65533, 65532, 65530, 65528, 65525, 65522, 65519,
65515, 65511, 65506, 65501, 65496, 65490, 65484, 65477, 65470, 65463,
65455, 65447, 65438, 65429, 65420, 65410, 65400, 65390, 65379, 65367,
65355, 65343, 65331, 65318, 65304, 65291, 65277, 65262, 65247, 65232,
65216, 65200, 65183, 65167, 65149, 65132, 65113, 65095, 65076, 65057,
65037, 65017, 64997, 64976, 64955, 64933, 64911, 64889, 64866, 64843,
64819, 64795, 64771, 64746, 64721, 64695, 64669, 64643, 64616, 64589,
64562, 64534, 64506, 64477, 64448, 64418, 64389, 64358, 64328, 64297,
64266, 64234, 64202, 64169, 64136, 64103, 64070, 64036, 64001, 63966,
63931, 63896, 63860, 63824, 63787, 63750, 63712, 63675, 63636, 63598,
63559, 63520, 63480, 63440, 63399, 63359, 63317, 63276, 63234, 63192,
63149, 63106, 63063, 63019, 62975, 62930, 62885, 62840, 62794, 62748,
62702, 62655, 62608, 62561, 62513, 62465, 62416, 62368, 62318, 62269,
62219, 62168, 62118, 62067, 62015, 61964, 61911, 61859, 61806, 61753,
61699, 61646, 61591, 61537, 61482, 61427, 61371, 61315, 61259, 61202,
61145, 61088, 61030, 60972, 60913, 60855, 60796, 60736, 60676, 60616,
60556, 60495, 60434, 60373, 60311, 60249, 60186, 60123, 60060, 59997,
59933, 59869, 59804, 59740, 59675, 59609, 59543, 59477, 59411, 59344,
59277, 59210, 59142, 59074, 59005, 58937, 58868, 58798, 58729, 58659,
58589, 58518, 58447, 58376, 58304, 58233, 58160, 58088, 58015, 57942,
57869, 57795, 57721, 57647, 57572, 57497, 57422, 57347, 57271, 57195,
57118, 57042, 56965, 56888, 56810, 56732, 56654, 56576, 56497, 56418,
56338, 56259, 56179, 56099, 56018, 55938, 55857, 55775, 55694, 55612,
55530, 55447, 55365, 55282, 55198, 55115, 55031, 54947, 54863, 54778,
54693, 54608, 54523, 54437, 54351, 54265, 54178, 54092, 54005, 53918,
53830, 53742, 53654, 53566, 53478, 53389, 53300, 53210, 53121, 53031,
52941, 52851, 52760, 52670, 52579, 52487, 52396, 52304, 52212, 52120,
52028, 51935, 51842, 51749, 51656, 51562, 51468, 51374, 51280, 51186,
51091, 50996, 50901, 50805, 50710, 50614, 50518, 50422, 50325, 50229,
50132, 50035, 49937, 49840, 49742, 49644, 49546, 49448, 49349, 49250,
49151, 49052, 48953, 48853, 48753, 48653, 48553, 48453, 48353, 48252,
48151, 48050, 47949, 47847, 47745, 47644, 47542, 47439, 47337, 47235,
47132, 47029, 46926, 46823, 46719, 46616, 46512, 46408, 46304, 46200,
46095, 45991, 45886, 45781, 45676, 45571, 45465, 45360, 45254, 45148,
45042, 44936, 44830, 44724, 44617, 44510, 44403, 44296, 44189, 44082,
43975, 43867, 43759, 43652, 43544, 43436, 43327, 43219, 43111, 43002,
42893, 42784, 42675, 42566, 42457, 42348, 42238, 42129, 42019, 41909,
41799, 41689, 41579, 41469, 41359, 41248, 41138, 41027, 40916, 40806,
40695, 40584, 40473, 40361, 40250, 40139, 40027, 39916, 39804, 39692,
39580, 39468, 39356, 39244, 39132, 39020, 38908, 38795, 38683, 38570,
38458, 38345, 38232, 38119, 38006, 37893, 37780, 37667, 37554, 37441,
37328, 37215, 37101, 36988, 36874, 36761, 36647, 36534, 36420, 36306,
36193, 36079, 35965, 35851, 35737, 35623, 35509, 35395, 35281, 35167,
35053, 34939, 34825, 34711, 34597, 34482, 34368, 34254, 34140, 34025,
33911, 33797, 33682, 33568, 33454, 33339, 33225, 33111, 32996, 32882,
32768, 32653, 32539, 32424, 32310, 32196, 32081, 31967, 31853, 31738,
31624, 31510, 31395, 31281, 31167, 31053, 30938, 30824, 30710, 30596,
30482, 30368, 30254, 30140, 30026, 29912, 29798, 29684, 29570, 29456,
29342, 29229, 29115, 29001, 28888, 28774, 28661, 28547, 28434, 28320,
28207, 28094, 27981, 27868, 27755, 27642, 27529, 27416, 27303, 27190,
27077, 26965, 26852, 26740, 26627, 26515, 26403, 26291, 26179, 26067,
25955, 25843, 25731, 25619, 25508, 25396, 25285, 25174, 25062, 24951,
24840, 24729, 24619, 24508, 24397, 24287, 24176, 24066, 23956, 23846,
23736, 23626, 23516, 23406, 23297, 23187, 23078, 22969, 22860, 22751,
22642, 22533, 22424, 22316, 22208, 22099, 21991, 21883, 21776, 21668,
21560, 21453, 21346, 21239, 21132, 21025, 20918, 20811, 20705, 20599,
20493, 20387, 20281, 20175, 20070, 19964, 19859, 19754, 19649, 19544,
19440, 19335, 19231, 19127, 19023, 18919, 18816, 18712, 18609, 18506,
18403, 18300, 18198, 18096, 17993, 17891, 17790, 17688, 17586, 17485,
17384, 17283, 17182, 17082, 16982, 16882, 16782, 16682, 16582, 16483,
16384, 16285, 16186, 16087, 15989, 15891, 15793, 15695, 15598, 15500,
15403, 15306, 15210, 15113, 15017, 14921, 14825, 14730, 14634, 14539,
14444, 14349, 14255, 14161, 14067, 13973, 13879, 13786, 13693, 13600,
13507, 13415, 13323, 13231, 13139, 13048, 12956, 12865, 12775, 12684,
12594, 12504, 12414, 12325, 12235, 12146, 12057, 11969, 11881, 11793,
11705, 11617, 11530, 11443, 11357, 11270, 11184, 11098, 11012, 10927,
10842, 10757, 10672, 10588, 10504, 10420, 10337, 10253, 10170, 10088,
10005, 9923, 9841, 9760, 9678, 9597, 9517, 9436, 9356, 9276,
9197, 9117, 9038, 8959, 8881, 8803, 8725, 8647, 8570, 8493,
8417, 8340, 8264, 8188, 8113, 8038, 7963, 7888, 7814, 7740,
7666, 7593, 7520, 7447, 7375, 7302, 7231, 7159, 7088, 7017,
6946, 6876, 6806, 6737, 6667, 6598, 6530, 6461, 6393, 6325,
6258, 6191, 6124, 6058, 5992, 5926, 5860, 5795, 5731, 5666,
5602, 5538, 5475, 5412, 5349, 5286, 5224, 5162, 5101, 5040,
4979, 4919, 4859, 4799, 4739, 4680, 4622, 4563, 4505, 4447,
4390, 4333, 4276, 4220, 4164, 4108, 4053, 3998, 3944, 3889,
3836, 3782, 3729, 3676, 3624, 3571, 3520, 3468, 3417, 3367,
3316, 3266, 3217, 3167, 3119, 3070, 3022, 2974, 2927, 2880,
2833, 2787, 2741, 2695, 2650, 2605, 2560, 2516, 2472, 2429,
2386, 2343, 2301, 2259, 2218, 2176, 2136, 2095, 2055, 2015,
1976, 1937, 1899, 1860, 1823, 1785, 1748, 1711, 1675, 1639,
1604, 1569, 1534, 1499, 1465, 1432, 1399, 1366, 1333, 1301,
1269, 1238, 1207, 1177, 1146, 1117, 1087, 1058, 1029, 1001,
973, 946, 919, 892, 866, 840, 814, 789, 764, 740,
716, 692, 669, 646, 624, 602, 580, 559, 538, 518,
498, 478, 459, 440, 422, 403, 386, 368, 352, 335,
319, 303, 288, 273, 258, 244, 231, 217, 204, 192,
180, 168, 156, 145, 135, 125, 115, 106, 97, 88,
80, 72, 65, 58, 51, 45, 39, 34, 29, 24,
20, 16, 13, 10, 7, 5, 3, 2, 1, 0,
0, 0, 1, 2, 3, 5, 7, 10, 13, 16,
20, 24, 29, 34, 39, 45, 51, 58, 65, 72,
80, 88, 97, 106, 115, 125, 135, 145, 156, 168,
180, 192, 204, 217, 231, 244, 258, 273, 288, 303,
319, 335, 352, 368, 386, 403, 422, 440, 459, 478,
498, 518, 538, 559, 580, 602, 624, 646, 669, 692,
716, 740, 764, 789, 814, 840, 866, 892, 919, 946,
973, 1001, 1029, 1058, 1087, 1117, 1146, 1177, 1207, 1238,
1269, 1301, 1333, 1366, 1399, 1432, 1465, 1499, 1534, 1569,
1604, 1639, 1675, 1711, 1748, 1785, 1823, 1860, 1899, 1937,
1976, 2015, 2055, 2095, 2136, 2176, 2218, 2259, 2301, 2343,
2386, 2429, 2472, 2516, 2560, 2605, 2650, 2695, 2741, 2787,
2833, 2880, 2927, 2974, 3022, 3070, 3119, 3167, 3217, 3266,
3316, 3367, 3417, 3468, 3520, 3571, 3624, 3676, 3729, 3782,
3836, 3889, 3944, 3998, 4053, 4108, 4164, 4220, 4276, 4333,
4390, 4447, 4505, 4563, 4622, 4680, 4739, 4799, 4859, 4919,
4979, 5040, 5101, 5162, 5224, 5286, 5349, 5412, 5475, 5538,
5602, 5666, 5731, 5795, 5860, 5926, 5992, 6058, 6124, 6191,
6258, 6325, 6393, 6461, 6530, 6598, 6667, 6737, 6806, 6876,
6946, 7017, 7088, 7159, 7231, 7302, 7375, 7447, 7520, 7593,
7666, 7740, 7814, 7888, 7963, 8038, 8113, 8188, 8264, 8340,
8417, 8493, 8570, 8647, 8725, 8803, 8881, 8959, 9038, 9117,
9197, 9276, 9356, 9436, 9517, 9597, 9678, 9760, 9841, 9923,
10005, 10088, 10170, 10253, 10337, 10420, 10504, 10588, 10672, 10757,
10842, 10927, 11012, 11098, 11184, 11270, 11357, 11443, 11530, 11617,
11705, 11793, 11881, 11969, 12057, 12146, 12235, 12325, 12414, 12504,
12594, 12684, 12775, 12865, 12956, 13048, 13139, 13231, 13323, 13415,
13507, 13600, 13693, 13786, 13879, 13973, 14067, 14161, 14255, 14349,
14444, 14539, 14634, 14730, 14825, 14921, 15017, 15113, 15210, 15306,
15403, 15500, 15598, 15695, 15793, 15891, 15989, 16087, 16186, 16285,
16384, 16483, 16582, 16682, 16782, 16882, 16982, 17082, 17182, 17283,
17384, 17485, 17586, 17688, 17790, 17891, 17993, 18096, 18198, 18300,
18403, 18506, 18609, 18712, 18816, 18919, 19023, 19127, 19231, 19335,
19440, 19544, 19649, 19754, 19859, 19964, 20070, 20175, 20281, 20387,
20493, 20599, 20705, 20811, 20918, 21025, 21132, 21239, 21346, 21453,
21560, 21668, 21776, 21883, 21991, 22099, 22208, 22316, 22424, 22533,
22642, 22751, 22860, 22969, 23078, 23187, 23297, 23406, 23516, 23626,
23736, 23846, 23956, 24066, 24176, 24287, 24397, 24508, 24619, 24729,
24840, 24951, 25062, 25174, 25285, 25396, 25508, 25619, 25731, 25843,
25955, 26067, 26179, 26291, 26403, 26515, 26627, 26740, 26852, 26965,
27077, 27190, 27303, 27416, 27529, 27642, 27755, 27868, 27981, 28094,
28207, 28320, 28434, 28547, 28661, 28774, 28888, 29001, 29115, 29229,
29342, 29456, 29570, 29684, 29798, 29912, 30026, 30140, 30254, 30368,
30482, 30596, 30710, 30824, 30938, 31053, 31167, 31281, 31395, 31510,
31624, 31738, 31853, 31967, 32081, 32196, 32310, 32424, 32539, 32653};

#endif