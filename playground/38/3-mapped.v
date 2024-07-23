// Benchmark "adder" written by ABC on Thu Jul 18 07:25:17 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n149, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n170,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n176, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n187, new_n188, new_n189, new_n190, new_n191, new_n192,
    new_n193, new_n195, new_n196, new_n197, new_n198, new_n199, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n225, new_n226, new_n227, new_n228, new_n229, new_n230,
    new_n231, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n243, new_n244, new_n245, new_n246, new_n247,
    new_n248, new_n249, new_n250, new_n251, new_n253, new_n254, new_n255,
    new_n256, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n269, new_n270,
    new_n271, new_n272, new_n273, new_n274, new_n276, new_n277, new_n278,
    new_n279, new_n280, new_n281, new_n282, new_n283, new_n284, new_n285,
    new_n286, new_n287, new_n288, new_n289, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n318, new_n319, new_n320, new_n321, new_n322, new_n323,
    new_n324, new_n325, new_n327, new_n328, new_n329, new_n330, new_n331,
    new_n332, new_n333, new_n334, new_n335, new_n336, new_n337, new_n338,
    new_n340, new_n341, new_n342, new_n343, new_n344, new_n345, new_n346,
    new_n347, new_n348, new_n349, new_n350, new_n352, new_n353, new_n354,
    new_n355, new_n356, new_n357, new_n358, new_n359, new_n360, new_n361,
    new_n364, new_n365, new_n366, new_n367, new_n368, new_n369, new_n370,
    new_n371, new_n372, new_n373, new_n374, new_n376, new_n377, new_n378,
    new_n379, new_n380, new_n381, new_n382, new_n383, new_n385, new_n386,
    new_n387, new_n388, new_n390, new_n392, new_n393, new_n394, new_n396,
    new_n397, new_n398, new_n400, new_n401, new_n402, new_n403, new_n405,
    new_n407, new_n408;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  and002aa1n12x5               g001(.a(\b[0] ), .b(\a[1] ), .o(new_n97));
  oaoi03aa1n12x5               g002(.a(\a[2] ), .b(\b[1] ), .c(new_n97), .o1(new_n98));
  nor002aa1n06x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nand42aa1d28x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  nor002aa1d32x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nand42aa1n08x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nano23aa1n02x5               g007(.a(new_n99), .b(new_n101), .c(new_n102), .d(new_n100), .out0(new_n103));
  aoi012aa1n06x5               g008(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n104));
  inv020aa1n02x5               g009(.a(new_n104), .o1(new_n105));
  aoi012aa1n02x5               g010(.a(new_n105), .b(new_n103), .c(new_n98), .o1(new_n106));
  nand42aa1d28x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  oai012aa1n02x5               g012(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .o1(new_n108));
  tech160nm_finand02aa1n03p5x5 g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  oai012aa1n03x5               g014(.a(new_n109), .b(\b[5] ), .c(\a[6] ), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand22aa1n09x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  norb02aa1n03x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nor002aa1d24x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand22aa1n09x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  norb02aa1n15x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nona23aa1d18x5               g021(.a(new_n113), .b(new_n116), .c(new_n110), .d(new_n108), .out0(new_n117));
  nano23aa1n06x5               g022(.a(new_n111), .b(new_n114), .c(new_n115), .d(new_n112), .out0(new_n118));
  nor002aa1d32x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  inv040aa1n06x5               g024(.a(new_n119), .o1(new_n120));
  oaoi03aa1n09x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  inv030aa1n02x5               g026(.a(new_n114), .o1(new_n122));
  tech160nm_fioaoi03aa1n05x5   g027(.a(\a[8] ), .b(\b[7] ), .c(new_n122), .o1(new_n123));
  aoi012aa1n12x5               g028(.a(new_n123), .b(new_n118), .c(new_n121), .o1(new_n124));
  xnrc02aa1n12x5               g029(.a(\b[8] ), .b(\a[9] ), .out0(new_n125));
  oaoi13aa1n02x5               g030(.a(new_n125), .b(new_n124), .c(new_n106), .d(new_n117), .o1(new_n126));
  nor042aa1d18x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n06x4               g032(.a(new_n100), .b(new_n99), .out0(new_n128));
  norb02aa1n06x4               g033(.a(new_n102), .b(new_n101), .out0(new_n129));
  nand23aa1n06x5               g034(.a(new_n98), .b(new_n128), .c(new_n129), .o1(new_n130));
  aoai13aa1n12x5               g035(.a(new_n124), .b(new_n117), .c(new_n130), .d(new_n104), .o1(new_n131));
  inv000aa1n02x5               g036(.a(new_n125), .o1(new_n132));
  aoi012aa1n02x5               g037(.a(new_n127), .b(new_n131), .c(new_n132), .o1(new_n133));
  nor042aa1d18x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  nand02aa1d28x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  inv000aa1d42x5               g041(.a(new_n134), .o1(new_n137));
  oai112aa1n02x5               g042(.a(new_n137), .b(new_n135), .c(\b[8] ), .d(\a[9] ), .o1(new_n138));
  oai022aa1n02x5               g043(.a(new_n133), .b(new_n136), .c(new_n138), .d(new_n126), .o1(\s[10] ));
  nano22aa1n03x7               g044(.a(new_n125), .b(new_n137), .c(new_n135), .out0(new_n140));
  nanp02aa1n02x5               g045(.a(new_n131), .b(new_n140), .o1(new_n141));
  nor002aa1d32x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n142), .o1(new_n143));
  nand02aa1d16x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  oai112aa1n02x5               g049(.a(new_n143), .b(new_n144), .c(\b[9] ), .d(\a[10] ), .o1(new_n145));
  aoi013aa1n02x4               g050(.a(new_n145), .b(new_n135), .c(new_n137), .d(new_n127), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n141), .b(new_n146), .o1(new_n147));
  aoi012aa1d18x5               g052(.a(new_n134), .b(new_n127), .c(new_n135), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n144), .b(new_n142), .out0(new_n149));
  aoai13aa1n02x5               g054(.a(new_n147), .b(new_n149), .c(new_n148), .d(new_n141), .o1(\s[11] ));
  inv020aa1n03x5               g055(.a(new_n148), .o1(new_n151));
  aoai13aa1n04x5               g056(.a(new_n149), .b(new_n151), .c(new_n131), .d(new_n140), .o1(new_n152));
  nor002aa1n20x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  nand02aa1d28x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  norb02aa1n02x5               g059(.a(new_n154), .b(new_n153), .out0(new_n155));
  nona23aa1n03x5               g060(.a(new_n152), .b(new_n154), .c(new_n153), .d(new_n142), .out0(new_n156));
  aoai13aa1n03x5               g061(.a(new_n156), .b(new_n155), .c(new_n143), .d(new_n152), .o1(\s[12] ));
  nano32aa1n02x4               g062(.a(new_n125), .b(new_n155), .c(new_n136), .d(new_n149), .out0(new_n158));
  nona23aa1n03x5               g063(.a(new_n154), .b(new_n144), .c(new_n142), .d(new_n153), .out0(new_n159));
  nanb03aa1n02x5               g064(.a(new_n153), .b(new_n154), .c(new_n142), .out0(new_n160));
  nor002aa1d32x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand02aa1d24x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  norb03aa1n02x5               g067(.a(new_n162), .b(new_n153), .c(new_n161), .out0(new_n163));
  oai112aa1n02x5               g068(.a(new_n160), .b(new_n163), .c(new_n159), .d(new_n148), .o1(new_n164));
  aoi012aa1n02x7               g069(.a(new_n153), .b(new_n142), .c(new_n154), .o1(new_n165));
  tech160nm_fioai012aa1n05x5   g070(.a(new_n165), .b(new_n159), .c(new_n148), .o1(new_n166));
  nanb02aa1n02x5               g071(.a(new_n161), .b(new_n162), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n166), .c(new_n131), .d(new_n158), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n168), .b(new_n164), .c(new_n131), .d(new_n158), .o1(\s[13] ));
  inv000aa1d42x5               g074(.a(new_n161), .o1(new_n170));
  inv000aa1d42x5               g075(.a(new_n167), .o1(new_n171));
  aoai13aa1n04x5               g076(.a(new_n171), .b(new_n166), .c(new_n131), .d(new_n158), .o1(new_n172));
  nor002aa1d32x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nand02aa1d28x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  nona23aa1n03x5               g080(.a(new_n172), .b(new_n174), .c(new_n173), .d(new_n161), .out0(new_n176));
  aoai13aa1n03x5               g081(.a(new_n176), .b(new_n175), .c(new_n170), .d(new_n172), .o1(\s[14] ));
  nano23aa1n09x5               g082(.a(new_n142), .b(new_n153), .c(new_n154), .d(new_n144), .out0(new_n178));
  nona23aa1n02x4               g083(.a(new_n174), .b(new_n162), .c(new_n161), .d(new_n173), .out0(new_n179));
  nano32aa1n02x4               g084(.a(new_n179), .b(new_n132), .c(new_n178), .d(new_n136), .out0(new_n180));
  nano23aa1n06x5               g085(.a(new_n161), .b(new_n173), .c(new_n174), .d(new_n162), .out0(new_n181));
  nano22aa1n02x4               g086(.a(new_n173), .b(new_n161), .c(new_n174), .out0(new_n182));
  nor002aa1d32x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  nanp02aa1n04x5               g089(.a(\b[14] ), .b(\a[15] ), .o1(new_n185));
  oai112aa1n02x5               g090(.a(new_n184), .b(new_n185), .c(\b[13] ), .d(\a[14] ), .o1(new_n186));
  aoi112aa1n02x5               g091(.a(new_n186), .b(new_n182), .c(new_n166), .d(new_n181), .o1(new_n187));
  aob012aa1n02x5               g092(.a(new_n187), .b(new_n131), .c(new_n180), .out0(new_n188));
  aobi12aa1n02x5               g093(.a(new_n165), .b(new_n178), .c(new_n151), .out0(new_n189));
  aoi012aa1n02x5               g094(.a(new_n173), .b(new_n161), .c(new_n174), .o1(new_n190));
  oai012aa1n02x5               g095(.a(new_n190), .b(new_n189), .c(new_n179), .o1(new_n191));
  aoi012aa1n02x5               g096(.a(new_n191), .b(new_n131), .c(new_n180), .o1(new_n192));
  norb02aa1n03x5               g097(.a(new_n185), .b(new_n183), .out0(new_n193));
  oai012aa1n02x5               g098(.a(new_n188), .b(new_n192), .c(new_n193), .o1(\s[15] ));
  aoai13aa1n04x5               g099(.a(new_n193), .b(new_n191), .c(new_n131), .d(new_n180), .o1(new_n195));
  nor022aa1n16x5               g100(.a(\b[15] ), .b(\a[16] ), .o1(new_n196));
  nand42aa1n06x5               g101(.a(\b[15] ), .b(\a[16] ), .o1(new_n197));
  norb02aa1n03x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  nona23aa1n03x5               g103(.a(new_n195), .b(new_n197), .c(new_n196), .d(new_n183), .out0(new_n199));
  aoai13aa1n03x5               g104(.a(new_n199), .b(new_n198), .c(new_n184), .d(new_n195), .o1(\s[16] ));
  tech160nm_fiaoi012aa1n05x5   g105(.a(new_n119), .b(\a[6] ), .c(\b[5] ), .o1(new_n201));
  nor042aa1n03x5               g106(.a(\b[5] ), .b(\a[6] ), .o1(new_n202));
  tech160nm_fiaoi012aa1n05x5   g107(.a(new_n202), .b(\a[5] ), .c(\b[4] ), .o1(new_n203));
  nand02aa1n04x5               g108(.a(new_n203), .b(new_n201), .o1(new_n204));
  nona23aa1n02x4               g109(.a(new_n115), .b(new_n112), .c(new_n111), .d(new_n114), .out0(new_n205));
  nor002aa1n02x5               g110(.a(new_n205), .b(new_n204), .o1(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n105), .c(new_n98), .d(new_n103), .o1(new_n207));
  nona23aa1n03x5               g112(.a(new_n197), .b(new_n185), .c(new_n183), .d(new_n196), .out0(new_n208));
  nona23aa1n03x5               g113(.a(new_n181), .b(new_n140), .c(new_n208), .d(new_n159), .out0(new_n209));
  aoi012aa1n02x5               g114(.a(new_n209), .b(new_n207), .c(new_n124), .o1(new_n210));
  norp02aa1n02x5               g115(.a(new_n208), .b(new_n179), .o1(new_n211));
  aoi012aa1n02x5               g116(.a(new_n196), .b(new_n183), .c(new_n197), .o1(new_n212));
  oai012aa1n02x5               g117(.a(new_n212), .b(new_n208), .c(new_n190), .o1(new_n213));
  aoi012aa1n03x5               g118(.a(new_n213), .b(new_n166), .c(new_n211), .o1(new_n214));
  aoai13aa1n04x5               g119(.a(new_n214), .b(new_n209), .c(new_n207), .d(new_n124), .o1(new_n215));
  nor002aa1d32x5               g120(.a(\b[16] ), .b(\a[17] ), .o1(new_n216));
  nand42aa1n06x5               g121(.a(\b[16] ), .b(\a[17] ), .o1(new_n217));
  norb02aa1n03x5               g122(.a(new_n217), .b(new_n216), .out0(new_n218));
  nanp03aa1n02x5               g123(.a(new_n181), .b(new_n193), .c(new_n198), .o1(new_n219));
  nanb03aa1n02x5               g124(.a(new_n196), .b(new_n197), .c(new_n183), .out0(new_n220));
  inv000aa1d42x5               g125(.a(new_n216), .o1(new_n221));
  nano32aa1n02x4               g126(.a(new_n196), .b(new_n217), .c(new_n220), .d(new_n221), .out0(new_n222));
  oai122aa1n02x7               g127(.a(new_n222), .b(new_n189), .c(new_n219), .d(new_n208), .e(new_n190), .o1(new_n223));
  obai22aa1n02x7               g128(.a(new_n215), .b(new_n218), .c(new_n210), .d(new_n223), .out0(\s[17] ));
  nano32aa1n03x7               g129(.a(new_n219), .b(new_n178), .c(new_n132), .d(new_n136), .out0(new_n225));
  oabi12aa1n06x5               g130(.a(new_n213), .b(new_n189), .c(new_n219), .out0(new_n226));
  aoai13aa1n04x5               g131(.a(new_n218), .b(new_n226), .c(new_n131), .d(new_n225), .o1(new_n227));
  nor002aa1n16x5               g132(.a(\b[17] ), .b(\a[18] ), .o1(new_n228));
  nand02aa1d28x5               g133(.a(\b[17] ), .b(\a[18] ), .o1(new_n229));
  norb02aa1n02x7               g134(.a(new_n229), .b(new_n228), .out0(new_n230));
  nona23aa1n03x5               g135(.a(new_n227), .b(new_n229), .c(new_n228), .d(new_n216), .out0(new_n231));
  aoai13aa1n03x5               g136(.a(new_n231), .b(new_n230), .c(new_n221), .d(new_n227), .o1(\s[18] ));
  nano23aa1n02x5               g137(.a(new_n216), .b(new_n228), .c(new_n229), .d(new_n217), .out0(new_n233));
  aoai13aa1n04x5               g138(.a(new_n233), .b(new_n226), .c(new_n131), .d(new_n225), .o1(new_n234));
  nano22aa1n02x4               g139(.a(new_n228), .b(new_n216), .c(new_n229), .out0(new_n235));
  nor042aa1n06x5               g140(.a(\b[18] ), .b(\a[19] ), .o1(new_n236));
  aoi012aa1n02x5               g141(.a(new_n228), .b(\a[19] ), .c(\b[18] ), .o1(new_n237));
  nona23aa1n03x5               g142(.a(new_n234), .b(new_n237), .c(new_n236), .d(new_n235), .out0(new_n238));
  aoi012aa1d24x5               g143(.a(new_n228), .b(new_n216), .c(new_n229), .o1(new_n239));
  xorc02aa1n12x5               g144(.a(\a[19] ), .b(\b[18] ), .out0(new_n240));
  aoai13aa1n03x5               g145(.a(new_n238), .b(new_n240), .c(new_n239), .d(new_n234), .o1(\s[19] ));
  xnrc02aa1n02x5               g146(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n02x5               g147(.a(new_n236), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n239), .o1(new_n244));
  aoai13aa1n02x5               g149(.a(new_n240), .b(new_n244), .c(new_n215), .d(new_n233), .o1(new_n245));
  nor042aa1n04x5               g150(.a(\b[19] ), .b(\a[20] ), .o1(new_n246));
  and002aa1n12x5               g151(.a(\b[19] ), .b(\a[20] ), .o(new_n247));
  nor042aa1n04x5               g152(.a(new_n247), .b(new_n246), .o1(new_n248));
  xnrc02aa1n12x5               g153(.a(\b[18] ), .b(\a[19] ), .out0(new_n249));
  norp03aa1n02x5               g154(.a(new_n247), .b(new_n246), .c(new_n236), .o1(new_n250));
  aoai13aa1n03x5               g155(.a(new_n250), .b(new_n249), .c(new_n234), .d(new_n239), .o1(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n248), .c(new_n245), .d(new_n243), .o1(\s[20] ));
  nano32aa1n03x7               g157(.a(new_n249), .b(new_n248), .c(new_n218), .d(new_n230), .out0(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n226), .c(new_n131), .d(new_n225), .o1(new_n254));
  norp03aa1n02x5               g159(.a(new_n243), .b(new_n247), .c(new_n246), .o1(new_n255));
  nor002aa1d32x5               g160(.a(\b[20] ), .b(\a[21] ), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  nand02aa1d28x5               g162(.a(\b[20] ), .b(\a[21] ), .o1(new_n258));
  oai112aa1n02x5               g163(.a(new_n257), .b(new_n258), .c(\b[19] ), .d(\a[20] ), .o1(new_n259));
  aoi113aa1n02x5               g164(.a(new_n255), .b(new_n259), .c(new_n244), .d(new_n240), .e(new_n248), .o1(new_n260));
  nanp02aa1n02x5               g165(.a(new_n254), .b(new_n260), .o1(new_n261));
  tech160nm_fixnrc02aa1n04x5   g166(.a(\b[19] ), .b(\a[20] ), .out0(new_n262));
  oab012aa1n09x5               g167(.a(new_n246), .b(new_n243), .c(new_n247), .out0(new_n263));
  oai013aa1d12x5               g168(.a(new_n263), .b(new_n249), .c(new_n262), .d(new_n239), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  nanb02aa1n12x5               g170(.a(new_n256), .b(new_n258), .out0(new_n266));
  inv000aa1n02x5               g171(.a(new_n266), .o1(new_n267));
  aoai13aa1n03x5               g172(.a(new_n261), .b(new_n267), .c(new_n265), .d(new_n254), .o1(\s[21] ));
  aoai13aa1n02x5               g173(.a(new_n267), .b(new_n264), .c(new_n215), .d(new_n253), .o1(new_n269));
  nor002aa1d32x5               g174(.a(\b[21] ), .b(\a[22] ), .o1(new_n270));
  nand02aa1d28x5               g175(.a(\b[21] ), .b(\a[22] ), .o1(new_n271));
  norb02aa1n02x5               g176(.a(new_n271), .b(new_n270), .out0(new_n272));
  norb03aa1n02x5               g177(.a(new_n271), .b(new_n256), .c(new_n270), .out0(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n266), .c(new_n254), .d(new_n265), .o1(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n272), .c(new_n269), .d(new_n257), .o1(\s[22] ));
  nona23aa1d18x5               g180(.a(new_n271), .b(new_n258), .c(new_n256), .d(new_n270), .out0(new_n276));
  nano32aa1n02x4               g181(.a(new_n276), .b(new_n233), .c(new_n240), .d(new_n248), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n226), .c(new_n131), .d(new_n225), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n276), .o1(new_n279));
  tech160nm_fiaoi012aa1n04x5   g184(.a(new_n270), .b(new_n256), .c(new_n271), .o1(new_n280));
  inv020aa1n03x5               g185(.a(new_n280), .o1(new_n281));
  aoi012aa1n02x5               g186(.a(new_n281), .b(new_n264), .c(new_n279), .o1(new_n282));
  norp02aa1n24x5               g187(.a(\b[22] ), .b(\a[23] ), .o1(new_n283));
  nand22aa1n12x5               g188(.a(\b[22] ), .b(\a[23] ), .o1(new_n284));
  norb02aa1n12x5               g189(.a(new_n284), .b(new_n283), .out0(new_n285));
  nanb03aa1n02x5               g190(.a(new_n270), .b(new_n271), .c(new_n256), .out0(new_n286));
  inv030aa1n02x5               g191(.a(new_n283), .o1(new_n287));
  nano32aa1n02x4               g192(.a(new_n270), .b(new_n284), .c(new_n286), .d(new_n287), .out0(new_n288));
  oai112aa1n03x5               g193(.a(new_n278), .b(new_n288), .c(new_n276), .d(new_n265), .o1(new_n289));
  aoai13aa1n02x5               g194(.a(new_n289), .b(new_n285), .c(new_n278), .d(new_n282), .o1(\s[23] ));
  inv000aa1n02x5               g195(.a(new_n282), .o1(new_n291));
  aoai13aa1n02x5               g196(.a(new_n285), .b(new_n291), .c(new_n215), .d(new_n277), .o1(new_n292));
  nor042aa1n04x5               g197(.a(\b[23] ), .b(\a[24] ), .o1(new_n293));
  nand22aa1n09x5               g198(.a(\b[23] ), .b(\a[24] ), .o1(new_n294));
  norb02aa1n02x5               g199(.a(new_n294), .b(new_n293), .out0(new_n295));
  inv000aa1d42x5               g200(.a(new_n285), .o1(new_n296));
  norb03aa1n02x5               g201(.a(new_n294), .b(new_n283), .c(new_n293), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n296), .c(new_n278), .d(new_n282), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n295), .c(new_n292), .d(new_n287), .o1(\s[24] ));
  nano23aa1n06x5               g204(.a(new_n283), .b(new_n293), .c(new_n294), .d(new_n284), .out0(new_n300));
  nand03aa1n02x5               g205(.a(new_n300), .b(new_n267), .c(new_n272), .o1(new_n301));
  nano32aa1n02x4               g206(.a(new_n301), .b(new_n233), .c(new_n240), .d(new_n248), .out0(new_n302));
  aoai13aa1n06x5               g207(.a(new_n302), .b(new_n226), .c(new_n131), .d(new_n225), .o1(new_n303));
  nano22aa1n06x5               g208(.a(new_n276), .b(new_n285), .c(new_n295), .out0(new_n304));
  nanb03aa1n02x5               g209(.a(new_n293), .b(new_n294), .c(new_n283), .out0(new_n305));
  nor002aa1d32x5               g210(.a(\b[24] ), .b(\a[25] ), .o1(new_n306));
  nand42aa1n06x5               g211(.a(\b[24] ), .b(\a[25] ), .o1(new_n307));
  nona23aa1n02x4               g212(.a(new_n305), .b(new_n307), .c(new_n306), .d(new_n293), .out0(new_n308));
  aoi122aa1n02x5               g213(.a(new_n308), .b(new_n281), .c(new_n300), .d(new_n264), .e(new_n304), .o1(new_n309));
  nanp02aa1n02x5               g214(.a(new_n303), .b(new_n309), .o1(new_n310));
  nanb03aa1n06x5               g215(.a(new_n239), .b(new_n240), .c(new_n248), .out0(new_n311));
  oaoi03aa1n02x5               g216(.a(\a[24] ), .b(\b[23] ), .c(new_n287), .o1(new_n312));
  tech160nm_fiaoi012aa1n05x5   g217(.a(new_n312), .b(new_n300), .c(new_n281), .o1(new_n313));
  aoai13aa1n12x5               g218(.a(new_n313), .b(new_n301), .c(new_n311), .d(new_n263), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n314), .o1(new_n315));
  norb02aa1n02x5               g220(.a(new_n307), .b(new_n306), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n310), .b(new_n316), .c(new_n315), .d(new_n303), .o1(\s[25] ));
  inv040aa1n06x5               g222(.a(new_n306), .o1(new_n318));
  aoai13aa1n02x5               g223(.a(new_n316), .b(new_n314), .c(new_n215), .d(new_n302), .o1(new_n319));
  nor042aa1n04x5               g224(.a(\b[25] ), .b(\a[26] ), .o1(new_n320));
  and002aa1n18x5               g225(.a(\b[25] ), .b(\a[26] ), .o(new_n321));
  norp02aa1n02x5               g226(.a(new_n321), .b(new_n320), .o1(new_n322));
  inv000aa1d42x5               g227(.a(new_n316), .o1(new_n323));
  norp03aa1n02x5               g228(.a(new_n321), .b(new_n320), .c(new_n306), .o1(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n323), .c(new_n303), .d(new_n315), .o1(new_n325));
  aoai13aa1n03x5               g230(.a(new_n325), .b(new_n322), .c(new_n319), .d(new_n318), .o1(\s[26] ));
  nano23aa1d12x5               g231(.a(new_n321), .b(new_n320), .c(new_n318), .d(new_n307), .out0(new_n327));
  and003aa1n06x5               g232(.a(new_n253), .b(new_n327), .c(new_n304), .o(new_n328));
  aoai13aa1n12x5               g233(.a(new_n328), .b(new_n226), .c(new_n131), .d(new_n225), .o1(new_n329));
  oab012aa1n02x4               g234(.a(new_n320), .b(new_n318), .c(new_n321), .out0(new_n330));
  aobi12aa1n18x5               g235(.a(new_n330), .b(new_n314), .c(new_n327), .out0(new_n331));
  nor002aa1d32x5               g236(.a(\b[26] ), .b(\a[27] ), .o1(new_n332));
  nand22aa1n03x5               g237(.a(\b[26] ), .b(\a[27] ), .o1(new_n333));
  norb02aa1n09x5               g238(.a(new_n333), .b(new_n332), .out0(new_n334));
  norb03aa1n02x5               g239(.a(new_n333), .b(new_n320), .c(new_n332), .out0(new_n335));
  oai013aa1n02x4               g240(.a(new_n335), .b(new_n318), .c(new_n321), .d(new_n320), .o1(new_n336));
  aoi012aa1n02x5               g241(.a(new_n336), .b(new_n314), .c(new_n327), .o1(new_n337));
  nanp02aa1n03x5               g242(.a(new_n329), .b(new_n337), .o1(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n334), .c(new_n329), .d(new_n331), .o1(\s[27] ));
  inv020aa1n03x5               g244(.a(new_n332), .o1(new_n340));
  nand02aa1n02x5               g245(.a(new_n264), .b(new_n304), .o1(new_n341));
  inv000aa1d42x5               g246(.a(new_n327), .o1(new_n342));
  aoai13aa1n02x5               g247(.a(new_n330), .b(new_n342), .c(new_n341), .d(new_n313), .o1(new_n343));
  aoai13aa1n02x5               g248(.a(new_n334), .b(new_n343), .c(new_n215), .d(new_n328), .o1(new_n344));
  nor002aa1n04x5               g249(.a(\b[27] ), .b(\a[28] ), .o1(new_n345));
  and002aa1n12x5               g250(.a(\b[27] ), .b(\a[28] ), .o(new_n346));
  norp02aa1n02x5               g251(.a(new_n346), .b(new_n345), .o1(new_n347));
  inv000aa1d42x5               g252(.a(new_n334), .o1(new_n348));
  norp03aa1n02x5               g253(.a(new_n346), .b(new_n345), .c(new_n332), .o1(new_n349));
  aoai13aa1n03x5               g254(.a(new_n349), .b(new_n348), .c(new_n329), .d(new_n331), .o1(new_n350));
  aoai13aa1n03x5               g255(.a(new_n350), .b(new_n347), .c(new_n344), .d(new_n340), .o1(\s[28] ));
  nano23aa1n03x7               g256(.a(new_n346), .b(new_n345), .c(new_n340), .d(new_n333), .out0(new_n352));
  aoai13aa1n02x5               g257(.a(new_n352), .b(new_n343), .c(new_n215), .d(new_n328), .o1(new_n353));
  inv000aa1d42x5               g258(.a(new_n352), .o1(new_n354));
  norp03aa1n02x5               g259(.a(new_n340), .b(new_n346), .c(new_n345), .o1(new_n355));
  norp02aa1n02x5               g260(.a(\b[28] ), .b(\a[29] ), .o1(new_n356));
  aoi012aa1n02x5               g261(.a(new_n345), .b(\a[29] ), .c(\b[28] ), .o1(new_n357));
  norb03aa1n02x5               g262(.a(new_n357), .b(new_n355), .c(new_n356), .out0(new_n358));
  aoai13aa1n03x5               g263(.a(new_n358), .b(new_n354), .c(new_n329), .d(new_n331), .o1(new_n359));
  oab012aa1n06x5               g264(.a(new_n345), .b(new_n340), .c(new_n346), .out0(new_n360));
  xorc02aa1n02x5               g265(.a(\a[29] ), .b(\b[28] ), .out0(new_n361));
  aoai13aa1n03x5               g266(.a(new_n359), .b(new_n361), .c(new_n353), .d(new_n360), .o1(\s[29] ));
  xnrb03aa1n02x5               g267(.a(new_n97), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g268(.a(new_n361), .b(new_n347), .c(new_n334), .o(new_n364));
  aoai13aa1n02x5               g269(.a(new_n364), .b(new_n343), .c(new_n215), .d(new_n328), .o1(new_n365));
  tech160nm_fioaoi03aa1n03p5x5 g270(.a(\a[29] ), .b(\b[28] ), .c(new_n360), .o1(new_n366));
  inv000aa1d42x5               g271(.a(new_n366), .o1(new_n367));
  norp02aa1n02x5               g272(.a(\b[29] ), .b(\a[30] ), .o1(new_n368));
  nanp02aa1n02x5               g273(.a(\b[29] ), .b(\a[30] ), .o1(new_n369));
  norb02aa1n02x5               g274(.a(new_n369), .b(new_n368), .out0(new_n370));
  inv000aa1d42x5               g275(.a(new_n364), .o1(new_n371));
  oai012aa1n02x5               g276(.a(new_n361), .b(new_n355), .c(new_n345), .o1(new_n372));
  nano23aa1n02x4               g277(.a(new_n356), .b(new_n368), .c(new_n372), .d(new_n369), .out0(new_n373));
  aoai13aa1n03x5               g278(.a(new_n373), .b(new_n371), .c(new_n329), .d(new_n331), .o1(new_n374));
  aoai13aa1n03x5               g279(.a(new_n374), .b(new_n370), .c(new_n365), .d(new_n367), .o1(\s[30] ));
  nano32aa1n03x7               g280(.a(new_n348), .b(new_n370), .c(new_n347), .d(new_n361), .out0(new_n376));
  aoai13aa1n02x5               g281(.a(new_n376), .b(new_n343), .c(new_n215), .d(new_n328), .o1(new_n377));
  xnrc02aa1n02x5               g282(.a(\b[30] ), .b(\a[31] ), .out0(new_n378));
  inv000aa1d42x5               g283(.a(new_n378), .o1(new_n379));
  inv000aa1d42x5               g284(.a(new_n376), .o1(new_n380));
  aoi112aa1n03x4               g285(.a(new_n368), .b(new_n378), .c(new_n366), .d(new_n370), .o1(new_n381));
  aoai13aa1n03x5               g286(.a(new_n381), .b(new_n380), .c(new_n329), .d(new_n331), .o1(new_n382));
  aoi012aa1n02x5               g287(.a(new_n368), .b(new_n366), .c(new_n369), .o1(new_n383));
  aoai13aa1n03x5               g288(.a(new_n382), .b(new_n379), .c(new_n377), .d(new_n383), .o1(\s[31] ));
  xnrc02aa1n02x5               g289(.a(\b[1] ), .b(\a[2] ), .out0(new_n385));
  inv000aa1d42x5               g290(.a(new_n101), .o1(new_n386));
  oai112aa1n02x5               g291(.a(new_n386), .b(new_n102), .c(\b[1] ), .d(\a[2] ), .o1(new_n387));
  oabi12aa1n02x5               g292(.a(new_n387), .b(new_n97), .c(new_n385), .out0(new_n388));
  oaib12aa1n02x5               g293(.a(new_n388), .b(new_n129), .c(new_n98), .out0(\s[3] ));
  nanp02aa1n02x5               g294(.a(new_n98), .b(new_n129), .o1(new_n390));
  xnbna2aa1n03x5               g295(.a(new_n128), .b(new_n390), .c(new_n386), .out0(\s[4] ));
  norb02aa1n02x5               g296(.a(new_n109), .b(new_n119), .out0(new_n392));
  nanb03aa1n02x5               g297(.a(new_n99), .b(new_n101), .c(new_n100), .out0(new_n393));
  nona23aa1n02x4               g298(.a(new_n393), .b(new_n109), .c(new_n119), .d(new_n99), .out0(new_n394));
  obai22aa1n02x7               g299(.a(new_n130), .b(new_n394), .c(new_n106), .d(new_n392), .out0(\s[5] ));
  norb02aa1n02x5               g300(.a(new_n107), .b(new_n202), .out0(new_n396));
  aoai13aa1n02x5               g301(.a(new_n392), .b(new_n105), .c(new_n103), .d(new_n98), .o1(new_n397));
  nona22aa1n02x4               g302(.a(new_n397), .b(new_n202), .c(new_n108), .out0(new_n398));
  aoai13aa1n02x5               g303(.a(new_n398), .b(new_n396), .c(new_n120), .d(new_n397), .o1(\s[6] ));
  inv000aa1d42x5               g304(.a(new_n116), .o1(new_n400));
  aoi112aa1n02x5               g305(.a(new_n400), .b(new_n202), .c(new_n107), .d(new_n119), .o1(new_n401));
  aoai13aa1n02x5               g306(.a(new_n401), .b(new_n204), .c(new_n130), .d(new_n104), .o1(new_n402));
  oabi12aa1n02x5               g307(.a(new_n121), .b(new_n106), .c(new_n204), .out0(new_n403));
  aob012aa1n02x5               g308(.a(new_n402), .b(new_n403), .c(new_n400), .out0(\s[7] ));
  nanp02aa1n02x5               g309(.a(new_n403), .b(new_n116), .o1(new_n405));
  xnbna2aa1n03x5               g310(.a(new_n113), .b(new_n405), .c(new_n122), .out0(\s[8] ));
  aoi112aa1n02x5               g311(.a(new_n125), .b(new_n111), .c(new_n112), .d(new_n114), .o1(new_n407));
  aobi12aa1n02x5               g312(.a(new_n407), .b(new_n121), .c(new_n118), .out0(new_n408));
  ao0022aa1n03x5               g313(.a(new_n131), .b(new_n125), .c(new_n408), .d(new_n207), .o(\s[9] ));
endmodule

