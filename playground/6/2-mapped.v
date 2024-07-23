// Benchmark "adder" written by ABC on Wed Jul 17 14:58:30 2024

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
    new_n186, new_n187, new_n188, new_n189, new_n190, new_n191, new_n193,
    new_n194, new_n195, new_n196, new_n197, new_n198, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n219, new_n220, new_n221, new_n222, new_n223,
    new_n224, new_n225, new_n227, new_n228, new_n229, new_n230, new_n231,
    new_n232, new_n233, new_n234, new_n237, new_n238, new_n239, new_n240,
    new_n241, new_n242, new_n243, new_n244, new_n245, new_n246, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n254, new_n255,
    new_n256, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n264, new_n265, new_n266, new_n267, new_n268, new_n269, new_n271,
    new_n272, new_n273, new_n274, new_n275, new_n276, new_n277, new_n278,
    new_n279, new_n280, new_n281, new_n282, new_n283, new_n284, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n313, new_n314, new_n315, new_n316,
    new_n317, new_n318, new_n319, new_n320, new_n322, new_n323, new_n324,
    new_n325, new_n326, new_n327, new_n328, new_n329, new_n330, new_n331,
    new_n332, new_n333, new_n335, new_n336, new_n337, new_n338, new_n339,
    new_n340, new_n341, new_n342, new_n343, new_n344, new_n345, new_n347,
    new_n348, new_n349, new_n350, new_n351, new_n352, new_n353, new_n354,
    new_n355, new_n358, new_n359, new_n360, new_n361, new_n362, new_n363,
    new_n364, new_n365, new_n366, new_n367, new_n368, new_n369, new_n371,
    new_n372, new_n373, new_n374, new_n375, new_n376, new_n377, new_n378,
    new_n380, new_n381, new_n382, new_n383, new_n384, new_n386, new_n388,
    new_n389, new_n390, new_n392, new_n393, new_n394, new_n396, new_n397,
    new_n398, new_n399, new_n400, new_n402, new_n403, new_n405, new_n406;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  and002aa1n12x5               g001(.a(\b[0] ), .b(\a[1] ), .o(new_n97));
  oaoi03aa1n12x5               g002(.a(\a[2] ), .b(\b[1] ), .c(new_n97), .o1(new_n98));
  nor042aa1n04x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nand02aa1d10x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  nor002aa1n16x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nanp02aa1n04x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nano23aa1n02x4               g007(.a(new_n99), .b(new_n101), .c(new_n102), .d(new_n100), .out0(new_n103));
  aoi012aa1n06x5               g008(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n104));
  inv000aa1n02x5               g009(.a(new_n104), .o1(new_n105));
  nor042aa1n06x5               g010(.a(\b[7] ), .b(\a[8] ), .o1(new_n106));
  nand42aa1d28x5               g011(.a(\b[7] ), .b(\a[8] ), .o1(new_n107));
  nor002aa1d32x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  nand42aa1n16x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nona23aa1n09x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  nor042aa1d18x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  aoi012aa1n03x5               g016(.a(new_n111), .b(\a[6] ), .c(\b[5] ), .o1(new_n112));
  nor002aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  aoi012aa1n02x5               g018(.a(new_n113), .b(\a[5] ), .c(\b[4] ), .o1(new_n114));
  nanp02aa1n03x5               g019(.a(new_n114), .b(new_n112), .o1(new_n115));
  nor002aa1n03x5               g020(.a(new_n110), .b(new_n115), .o1(new_n116));
  aoai13aa1n06x5               g021(.a(new_n116), .b(new_n105), .c(new_n98), .d(new_n103), .o1(new_n117));
  nano23aa1d15x5               g022(.a(new_n106), .b(new_n108), .c(new_n109), .d(new_n107), .out0(new_n118));
  inv030aa1n06x5               g023(.a(new_n111), .o1(new_n119));
  oaoi03aa1n09x5               g024(.a(\a[6] ), .b(\b[5] ), .c(new_n119), .o1(new_n120));
  inv040aa1n02x5               g025(.a(new_n108), .o1(new_n121));
  oaoi03aa1n12x5               g026(.a(\a[8] ), .b(\b[7] ), .c(new_n121), .o1(new_n122));
  aoi012aa1d18x5               g027(.a(new_n122), .b(new_n118), .c(new_n120), .o1(new_n123));
  xnrc02aa1n12x5               g028(.a(\b[8] ), .b(\a[9] ), .out0(new_n124));
  aoi012aa1n02x5               g029(.a(new_n124), .b(new_n117), .c(new_n123), .o1(new_n125));
  nor042aa1n04x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  norb02aa1n03x5               g031(.a(new_n100), .b(new_n99), .out0(new_n127));
  norb02aa1n06x4               g032(.a(new_n102), .b(new_n101), .out0(new_n128));
  nand23aa1n06x5               g033(.a(new_n98), .b(new_n127), .c(new_n128), .o1(new_n129));
  nand23aa1n03x5               g034(.a(new_n118), .b(new_n112), .c(new_n114), .o1(new_n130));
  aoai13aa1n12x5               g035(.a(new_n123), .b(new_n130), .c(new_n129), .d(new_n104), .o1(new_n131));
  inv000aa1d42x5               g036(.a(new_n124), .o1(new_n132));
  aoi012aa1n02x5               g037(.a(new_n126), .b(new_n131), .c(new_n132), .o1(new_n133));
  nor002aa1n10x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  nand02aa1n10x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  inv000aa1d42x5               g041(.a(new_n134), .o1(new_n137));
  oai112aa1n02x5               g042(.a(new_n137), .b(new_n135), .c(\b[8] ), .d(\a[9] ), .o1(new_n138));
  oai022aa1n02x5               g043(.a(new_n133), .b(new_n136), .c(new_n138), .d(new_n125), .o1(\s[10] ));
  nano22aa1n03x7               g044(.a(new_n124), .b(new_n137), .c(new_n135), .out0(new_n140));
  nanp02aa1n02x5               g045(.a(new_n131), .b(new_n140), .o1(new_n141));
  nanp02aa1n04x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  nor002aa1n16x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  oai112aa1n02x5               g049(.a(new_n144), .b(new_n142), .c(\b[9] ), .d(\a[10] ), .o1(new_n145));
  aoi013aa1n02x4               g050(.a(new_n145), .b(new_n135), .c(new_n137), .d(new_n126), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n141), .b(new_n146), .o1(new_n147));
  aoi012aa1n12x5               g052(.a(new_n134), .b(new_n126), .c(new_n135), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n142), .b(new_n143), .out0(new_n149));
  aoai13aa1n02x5               g054(.a(new_n147), .b(new_n149), .c(new_n148), .d(new_n141), .o1(\s[11] ));
  inv040aa1n02x5               g055(.a(new_n148), .o1(new_n151));
  aoai13aa1n04x5               g056(.a(new_n149), .b(new_n151), .c(new_n131), .d(new_n140), .o1(new_n152));
  nor002aa1n04x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  tech160nm_finand02aa1n05x5   g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  norb02aa1n02x5               g059(.a(new_n154), .b(new_n153), .out0(new_n155));
  nona23aa1n03x5               g060(.a(new_n152), .b(new_n154), .c(new_n153), .d(new_n143), .out0(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n155), .c(new_n144), .d(new_n152), .o1(\s[12] ));
  nano32aa1n02x4               g062(.a(new_n124), .b(new_n155), .c(new_n136), .d(new_n149), .out0(new_n158));
  nona23aa1n06x5               g063(.a(new_n142), .b(new_n154), .c(new_n153), .d(new_n143), .out0(new_n159));
  nanb03aa1n02x5               g064(.a(new_n153), .b(new_n154), .c(new_n143), .out0(new_n160));
  nand02aa1n03x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nor002aa1d32x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  norb03aa1n02x5               g067(.a(new_n161), .b(new_n153), .c(new_n162), .out0(new_n163));
  oai112aa1n02x5               g068(.a(new_n160), .b(new_n163), .c(new_n159), .d(new_n148), .o1(new_n164));
  tech160nm_fiaoi012aa1n04x5   g069(.a(new_n153), .b(new_n143), .c(new_n154), .o1(new_n165));
  tech160nm_fioai012aa1n05x5   g070(.a(new_n165), .b(new_n159), .c(new_n148), .o1(new_n166));
  nanb02aa1n02x5               g071(.a(new_n162), .b(new_n161), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n166), .c(new_n131), .d(new_n158), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n168), .b(new_n164), .c(new_n131), .d(new_n158), .o1(\s[13] ));
  inv000aa1d42x5               g074(.a(new_n162), .o1(new_n170));
  inv000aa1d42x5               g075(.a(new_n167), .o1(new_n171));
  aoai13aa1n04x5               g076(.a(new_n171), .b(new_n166), .c(new_n131), .d(new_n158), .o1(new_n172));
  nor022aa1n12x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nand02aa1d04x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  nona23aa1n03x5               g080(.a(new_n172), .b(new_n174), .c(new_n173), .d(new_n162), .out0(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n175), .c(new_n170), .d(new_n172), .o1(\s[14] ));
  nano23aa1n06x5               g082(.a(new_n153), .b(new_n143), .c(new_n154), .d(new_n142), .out0(new_n178));
  nona23aa1n02x4               g083(.a(new_n161), .b(new_n174), .c(new_n173), .d(new_n162), .out0(new_n179));
  nano32aa1n02x4               g084(.a(new_n179), .b(new_n132), .c(new_n178), .d(new_n136), .out0(new_n180));
  aobi12aa1n02x5               g085(.a(new_n180), .b(new_n117), .c(new_n123), .out0(new_n181));
  nanp02aa1n02x5               g086(.a(new_n178), .b(new_n151), .o1(new_n182));
  nanb03aa1n02x5               g087(.a(new_n173), .b(new_n174), .c(new_n162), .out0(new_n183));
  nand42aa1n06x5               g088(.a(\b[14] ), .b(\a[15] ), .o1(new_n184));
  nor042aa1n06x5               g089(.a(\b[14] ), .b(\a[15] ), .o1(new_n185));
  nano23aa1n02x4               g090(.a(new_n185), .b(new_n173), .c(new_n183), .d(new_n184), .out0(new_n186));
  aoai13aa1n02x5               g091(.a(new_n186), .b(new_n179), .c(new_n182), .d(new_n165), .o1(new_n187));
  aoi012aa1n02x5               g092(.a(new_n173), .b(new_n162), .c(new_n174), .o1(new_n188));
  aoai13aa1n02x5               g093(.a(new_n188), .b(new_n179), .c(new_n182), .d(new_n165), .o1(new_n189));
  tech160nm_fiaoi012aa1n05x5   g094(.a(new_n189), .b(new_n131), .c(new_n180), .o1(new_n190));
  norb02aa1n03x5               g095(.a(new_n184), .b(new_n185), .out0(new_n191));
  oai022aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n181), .d(new_n187), .o1(\s[15] ));
  oaoi13aa1n02x5               g097(.a(new_n185), .b(new_n184), .c(new_n181), .d(new_n189), .o1(new_n193));
  nor042aa1n04x5               g098(.a(\b[15] ), .b(\a[16] ), .o1(new_n194));
  nand42aa1n02x5               g099(.a(\b[15] ), .b(\a[16] ), .o1(new_n195));
  norb02aa1n06x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  norb03aa1n02x5               g101(.a(new_n195), .b(new_n185), .c(new_n194), .out0(new_n197));
  oaib12aa1n02x5               g102(.a(new_n197), .b(new_n190), .c(new_n191), .out0(new_n198));
  oai012aa1n02x5               g103(.a(new_n198), .b(new_n193), .c(new_n196), .o1(\s[16] ));
  nano23aa1n06x5               g104(.a(new_n173), .b(new_n162), .c(new_n174), .d(new_n161), .out0(new_n200));
  nona23aa1n06x5               g105(.a(new_n184), .b(new_n195), .c(new_n194), .d(new_n185), .out0(new_n201));
  nona23aa1n09x5               g106(.a(new_n200), .b(new_n140), .c(new_n201), .d(new_n159), .out0(new_n202));
  aoi012aa1n02x5               g107(.a(new_n202), .b(new_n117), .c(new_n123), .o1(new_n203));
  nor002aa1n02x5               g108(.a(new_n201), .b(new_n179), .o1(new_n204));
  aoi012aa1n02x5               g109(.a(new_n194), .b(new_n185), .c(new_n195), .o1(new_n205));
  oaih12aa1n02x5               g110(.a(new_n205), .b(new_n201), .c(new_n188), .o1(new_n206));
  tech160nm_fiaoi012aa1n05x5   g111(.a(new_n206), .b(new_n166), .c(new_n204), .o1(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n202), .c(new_n117), .d(new_n123), .o1(new_n208));
  nor002aa1d32x5               g113(.a(\b[16] ), .b(\a[17] ), .o1(new_n209));
  nand42aa1n06x5               g114(.a(\b[16] ), .b(\a[17] ), .o1(new_n210));
  norb02aa1n03x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  aobi12aa1n06x5               g116(.a(new_n165), .b(new_n178), .c(new_n151), .out0(new_n212));
  nand23aa1n06x5               g117(.a(new_n200), .b(new_n191), .c(new_n196), .o1(new_n213));
  nanb03aa1n02x5               g118(.a(new_n194), .b(new_n195), .c(new_n185), .out0(new_n214));
  inv000aa1d42x5               g119(.a(new_n209), .o1(new_n215));
  nano32aa1n02x4               g120(.a(new_n194), .b(new_n210), .c(new_n214), .d(new_n215), .out0(new_n216));
  oai122aa1n02x7               g121(.a(new_n216), .b(new_n212), .c(new_n213), .d(new_n201), .e(new_n188), .o1(new_n217));
  obai22aa1n02x7               g122(.a(new_n208), .b(new_n211), .c(new_n203), .d(new_n217), .out0(\s[17] ));
  nano32aa1d12x5               g123(.a(new_n213), .b(new_n178), .c(new_n132), .d(new_n136), .out0(new_n219));
  oabi12aa1n12x5               g124(.a(new_n206), .b(new_n212), .c(new_n213), .out0(new_n220));
  aoai13aa1n04x5               g125(.a(new_n211), .b(new_n220), .c(new_n131), .d(new_n219), .o1(new_n221));
  nor042aa1n06x5               g126(.a(\b[17] ), .b(\a[18] ), .o1(new_n222));
  nand02aa1d28x5               g127(.a(\b[17] ), .b(\a[18] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  nona23aa1n03x5               g129(.a(new_n221), .b(new_n223), .c(new_n222), .d(new_n209), .out0(new_n225));
  aoai13aa1n02x5               g130(.a(new_n225), .b(new_n224), .c(new_n215), .d(new_n221), .o1(\s[18] ));
  nano23aa1n02x4               g131(.a(new_n209), .b(new_n222), .c(new_n223), .d(new_n210), .out0(new_n227));
  aoai13aa1n06x5               g132(.a(new_n227), .b(new_n220), .c(new_n131), .d(new_n219), .o1(new_n228));
  nano22aa1n02x4               g133(.a(new_n222), .b(new_n209), .c(new_n223), .out0(new_n229));
  oai022aa1n02x5               g134(.a(\a[18] ), .b(\b[17] ), .c(\b[18] ), .d(\a[19] ), .o1(new_n230));
  aoi112aa1n02x5               g135(.a(new_n229), .b(new_n230), .c(\a[19] ), .d(\b[18] ), .o1(new_n231));
  nanp02aa1n02x5               g136(.a(new_n228), .b(new_n231), .o1(new_n232));
  aoi012aa1d24x5               g137(.a(new_n222), .b(new_n209), .c(new_n223), .o1(new_n233));
  xorc02aa1n12x5               g138(.a(\a[19] ), .b(\b[18] ), .out0(new_n234));
  aoai13aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n233), .d(new_n228), .o1(\s[19] ));
  xnrc02aa1n02x5               g140(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n02x5               g141(.a(\b[18] ), .b(\a[19] ), .o1(new_n237));
  inv000aa1n02x5               g142(.a(new_n237), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n233), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n234), .b(new_n239), .c(new_n208), .d(new_n227), .o1(new_n240));
  nor042aa1n04x5               g145(.a(\b[19] ), .b(\a[20] ), .o1(new_n241));
  and002aa1n12x5               g146(.a(\b[19] ), .b(\a[20] ), .o(new_n242));
  nor042aa1n06x5               g147(.a(new_n242), .b(new_n241), .o1(new_n243));
  tech160nm_fixnrc02aa1n04x5   g148(.a(\b[18] ), .b(\a[19] ), .out0(new_n244));
  norp03aa1n02x5               g149(.a(new_n242), .b(new_n241), .c(new_n237), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n245), .b(new_n244), .c(new_n228), .d(new_n233), .o1(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n243), .c(new_n240), .d(new_n238), .o1(\s[20] ));
  nano32aa1n03x7               g152(.a(new_n244), .b(new_n243), .c(new_n211), .d(new_n224), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n220), .c(new_n131), .d(new_n219), .o1(new_n249));
  norp03aa1n02x5               g154(.a(new_n238), .b(new_n242), .c(new_n241), .o1(new_n250));
  nand02aa1n10x5               g155(.a(\b[20] ), .b(\a[21] ), .o1(new_n251));
  nor002aa1d32x5               g156(.a(\b[20] ), .b(\a[21] ), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n252), .o1(new_n253));
  oai112aa1n02x5               g158(.a(new_n253), .b(new_n251), .c(\b[19] ), .d(\a[20] ), .o1(new_n254));
  aoi113aa1n02x5               g159(.a(new_n250), .b(new_n254), .c(new_n239), .d(new_n234), .e(new_n243), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(new_n249), .b(new_n255), .o1(new_n256));
  tech160nm_fixnrc02aa1n04x5   g161(.a(\b[19] ), .b(\a[20] ), .out0(new_n257));
  oab012aa1n12x5               g162(.a(new_n241), .b(new_n238), .c(new_n242), .out0(new_n258));
  oai013aa1d12x5               g163(.a(new_n258), .b(new_n244), .c(new_n257), .d(new_n233), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  nanb02aa1n06x5               g165(.a(new_n252), .b(new_n251), .out0(new_n261));
  inv000aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  aoai13aa1n02x5               g167(.a(new_n256), .b(new_n262), .c(new_n260), .d(new_n249), .o1(\s[21] ));
  aoai13aa1n02x5               g168(.a(new_n262), .b(new_n259), .c(new_n208), .d(new_n248), .o1(new_n264));
  nor002aa1n16x5               g169(.a(\b[21] ), .b(\a[22] ), .o1(new_n265));
  nand02aa1d28x5               g170(.a(\b[21] ), .b(\a[22] ), .o1(new_n266));
  norb02aa1n02x5               g171(.a(new_n266), .b(new_n265), .out0(new_n267));
  norb03aa1n02x5               g172(.a(new_n266), .b(new_n252), .c(new_n265), .out0(new_n268));
  aoai13aa1n03x5               g173(.a(new_n268), .b(new_n261), .c(new_n249), .d(new_n260), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n269), .b(new_n267), .c(new_n264), .d(new_n253), .o1(\s[22] ));
  nona23aa1d24x5               g175(.a(new_n251), .b(new_n266), .c(new_n265), .d(new_n252), .out0(new_n271));
  nano32aa1n03x5               g176(.a(new_n271), .b(new_n227), .c(new_n234), .d(new_n243), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n220), .c(new_n131), .d(new_n219), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n271), .o1(new_n274));
  tech160nm_fiaoi012aa1n05x5   g179(.a(new_n265), .b(new_n252), .c(new_n266), .o1(new_n275));
  inv000aa1n04x5               g180(.a(new_n275), .o1(new_n276));
  aoi012aa1d24x5               g181(.a(new_n276), .b(new_n259), .c(new_n274), .o1(new_n277));
  nor002aa1n20x5               g182(.a(\b[22] ), .b(\a[23] ), .o1(new_n278));
  nand02aa1d06x5               g183(.a(\b[22] ), .b(\a[23] ), .o1(new_n279));
  norb02aa1n15x5               g184(.a(new_n279), .b(new_n278), .out0(new_n280));
  nanb03aa1n02x5               g185(.a(new_n265), .b(new_n266), .c(new_n252), .out0(new_n281));
  inv000aa1d42x5               g186(.a(new_n278), .o1(new_n282));
  nano32aa1n02x4               g187(.a(new_n265), .b(new_n279), .c(new_n281), .d(new_n282), .out0(new_n283));
  oai112aa1n02x5               g188(.a(new_n273), .b(new_n283), .c(new_n271), .d(new_n260), .o1(new_n284));
  aoai13aa1n02x5               g189(.a(new_n284), .b(new_n280), .c(new_n273), .d(new_n277), .o1(\s[23] ));
  inv000aa1d42x5               g190(.a(new_n277), .o1(new_n286));
  aoai13aa1n03x5               g191(.a(new_n280), .b(new_n286), .c(new_n208), .d(new_n272), .o1(new_n287));
  nor042aa1n03x5               g192(.a(\b[23] ), .b(\a[24] ), .o1(new_n288));
  nand02aa1d04x5               g193(.a(\b[23] ), .b(\a[24] ), .o1(new_n289));
  norb02aa1n02x7               g194(.a(new_n289), .b(new_n288), .out0(new_n290));
  inv000aa1d42x5               g195(.a(new_n280), .o1(new_n291));
  norb03aa1n02x5               g196(.a(new_n289), .b(new_n278), .c(new_n288), .out0(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n291), .c(new_n273), .d(new_n277), .o1(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n290), .c(new_n287), .d(new_n282), .o1(\s[24] ));
  nano23aa1n06x5               g199(.a(new_n278), .b(new_n288), .c(new_n289), .d(new_n279), .out0(new_n295));
  nand23aa1n03x5               g200(.a(new_n295), .b(new_n262), .c(new_n267), .o1(new_n296));
  nano32aa1n03x5               g201(.a(new_n296), .b(new_n227), .c(new_n234), .d(new_n243), .out0(new_n297));
  aoai13aa1n06x5               g202(.a(new_n297), .b(new_n220), .c(new_n131), .d(new_n219), .o1(new_n298));
  nano22aa1n06x5               g203(.a(new_n271), .b(new_n280), .c(new_n290), .out0(new_n299));
  nanb03aa1n02x5               g204(.a(new_n288), .b(new_n289), .c(new_n278), .out0(new_n300));
  nand42aa1n03x5               g205(.a(\b[24] ), .b(\a[25] ), .o1(new_n301));
  nor022aa1n12x5               g206(.a(\b[24] ), .b(\a[25] ), .o1(new_n302));
  nona23aa1n02x4               g207(.a(new_n300), .b(new_n301), .c(new_n302), .d(new_n288), .out0(new_n303));
  aoi122aa1n02x5               g208(.a(new_n303), .b(new_n276), .c(new_n295), .d(new_n259), .e(new_n299), .o1(new_n304));
  nanp02aa1n02x5               g209(.a(new_n298), .b(new_n304), .o1(new_n305));
  nanb03aa1d24x5               g210(.a(new_n233), .b(new_n234), .c(new_n243), .out0(new_n306));
  oaoi03aa1n02x5               g211(.a(\a[24] ), .b(\b[23] ), .c(new_n282), .o1(new_n307));
  aoi012aa1n06x5               g212(.a(new_n307), .b(new_n295), .c(new_n276), .o1(new_n308));
  aoai13aa1n12x5               g213(.a(new_n308), .b(new_n296), .c(new_n306), .d(new_n258), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n309), .o1(new_n310));
  norb02aa1n02x5               g215(.a(new_n301), .b(new_n302), .out0(new_n311));
  aoai13aa1n02x5               g216(.a(new_n305), .b(new_n311), .c(new_n310), .d(new_n298), .o1(\s[25] ));
  inv000aa1d42x5               g217(.a(new_n302), .o1(new_n313));
  aoai13aa1n03x5               g218(.a(new_n311), .b(new_n309), .c(new_n208), .d(new_n297), .o1(new_n314));
  nor042aa1n02x5               g219(.a(\b[25] ), .b(\a[26] ), .o1(new_n315));
  and002aa1n06x5               g220(.a(\b[25] ), .b(\a[26] ), .o(new_n316));
  norp02aa1n02x5               g221(.a(new_n316), .b(new_n315), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n311), .o1(new_n318));
  norp03aa1n02x5               g223(.a(new_n316), .b(new_n315), .c(new_n302), .o1(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n318), .c(new_n298), .d(new_n310), .o1(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n317), .c(new_n314), .d(new_n313), .o1(\s[26] ));
  nano23aa1d12x5               g226(.a(new_n316), .b(new_n315), .c(new_n313), .d(new_n301), .out0(new_n322));
  and003aa1n12x5               g227(.a(new_n248), .b(new_n322), .c(new_n299), .o(new_n323));
  aoai13aa1n12x5               g228(.a(new_n323), .b(new_n220), .c(new_n131), .d(new_n219), .o1(new_n324));
  oab012aa1n02x4               g229(.a(new_n315), .b(new_n313), .c(new_n316), .out0(new_n325));
  aobi12aa1n12x5               g230(.a(new_n325), .b(new_n309), .c(new_n322), .out0(new_n326));
  nor042aa1n09x5               g231(.a(\b[26] ), .b(\a[27] ), .o1(new_n327));
  nand42aa1n02x5               g232(.a(\b[26] ), .b(\a[27] ), .o1(new_n328));
  norb02aa1n09x5               g233(.a(new_n328), .b(new_n327), .out0(new_n329));
  norb03aa1n02x5               g234(.a(new_n328), .b(new_n315), .c(new_n327), .out0(new_n330));
  oai013aa1n02x4               g235(.a(new_n330), .b(new_n313), .c(new_n316), .d(new_n315), .o1(new_n331));
  aoi012aa1n02x5               g236(.a(new_n331), .b(new_n309), .c(new_n322), .o1(new_n332));
  nanp02aa1n02x5               g237(.a(new_n324), .b(new_n332), .o1(new_n333));
  aoai13aa1n02x5               g238(.a(new_n333), .b(new_n329), .c(new_n324), .d(new_n326), .o1(\s[27] ));
  inv040aa1n02x5               g239(.a(new_n327), .o1(new_n335));
  nand02aa1d04x5               g240(.a(new_n259), .b(new_n299), .o1(new_n336));
  inv000aa1d42x5               g241(.a(new_n322), .o1(new_n337));
  aoai13aa1n04x5               g242(.a(new_n325), .b(new_n337), .c(new_n336), .d(new_n308), .o1(new_n338));
  aoai13aa1n02x5               g243(.a(new_n329), .b(new_n338), .c(new_n208), .d(new_n323), .o1(new_n339));
  nor042aa1n03x5               g244(.a(\b[27] ), .b(\a[28] ), .o1(new_n340));
  and002aa1n12x5               g245(.a(\b[27] ), .b(\a[28] ), .o(new_n341));
  nor002aa1n02x5               g246(.a(new_n341), .b(new_n340), .o1(new_n342));
  inv000aa1d42x5               g247(.a(new_n329), .o1(new_n343));
  norp03aa1n02x5               g248(.a(new_n341), .b(new_n340), .c(new_n327), .o1(new_n344));
  aoai13aa1n03x5               g249(.a(new_n344), .b(new_n343), .c(new_n324), .d(new_n326), .o1(new_n345));
  aoai13aa1n03x5               g250(.a(new_n345), .b(new_n342), .c(new_n339), .d(new_n335), .o1(\s[28] ));
  nano23aa1n02x4               g251(.a(new_n341), .b(new_n340), .c(new_n335), .d(new_n328), .out0(new_n347));
  aoai13aa1n06x5               g252(.a(new_n347), .b(new_n338), .c(new_n208), .d(new_n323), .o1(new_n348));
  inv000aa1n02x5               g253(.a(new_n347), .o1(new_n349));
  norp03aa1n02x5               g254(.a(new_n335), .b(new_n341), .c(new_n340), .o1(new_n350));
  oai022aa1n02x5               g255(.a(\a[28] ), .b(\b[27] ), .c(\b[28] ), .d(\a[29] ), .o1(new_n351));
  aoi112aa1n02x5               g256(.a(new_n350), .b(new_n351), .c(\a[29] ), .d(\b[28] ), .o1(new_n352));
  aoai13aa1n03x5               g257(.a(new_n352), .b(new_n349), .c(new_n324), .d(new_n326), .o1(new_n353));
  oab012aa1n06x5               g258(.a(new_n340), .b(new_n335), .c(new_n341), .out0(new_n354));
  xorc02aa1n02x5               g259(.a(\a[29] ), .b(\b[28] ), .out0(new_n355));
  aoai13aa1n03x5               g260(.a(new_n353), .b(new_n355), .c(new_n348), .d(new_n354), .o1(\s[29] ));
  xnrb03aa1n02x5               g261(.a(new_n97), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g262(.a(new_n355), .b(new_n342), .c(new_n329), .o(new_n358));
  aoai13aa1n02x5               g263(.a(new_n358), .b(new_n338), .c(new_n208), .d(new_n323), .o1(new_n359));
  tech160nm_fioaoi03aa1n03p5x5 g264(.a(\a[29] ), .b(\b[28] ), .c(new_n354), .o1(new_n360));
  inv000aa1d42x5               g265(.a(new_n360), .o1(new_n361));
  norp02aa1n02x5               g266(.a(\b[29] ), .b(\a[30] ), .o1(new_n362));
  nanp02aa1n02x5               g267(.a(\b[29] ), .b(\a[30] ), .o1(new_n363));
  norb02aa1n02x5               g268(.a(new_n363), .b(new_n362), .out0(new_n364));
  inv000aa1d42x5               g269(.a(new_n358), .o1(new_n365));
  oai012aa1n02x5               g270(.a(new_n355), .b(new_n350), .c(new_n340), .o1(new_n366));
  oai022aa1n02x5               g271(.a(\a[29] ), .b(\b[28] ), .c(\b[29] ), .d(\a[30] ), .o1(new_n367));
  nano22aa1n02x4               g272(.a(new_n367), .b(new_n366), .c(new_n363), .out0(new_n368));
  aoai13aa1n03x5               g273(.a(new_n368), .b(new_n365), .c(new_n324), .d(new_n326), .o1(new_n369));
  aoai13aa1n03x5               g274(.a(new_n369), .b(new_n364), .c(new_n359), .d(new_n361), .o1(\s[30] ));
  nano32aa1n03x7               g275(.a(new_n343), .b(new_n364), .c(new_n342), .d(new_n355), .out0(new_n371));
  aoai13aa1n03x5               g276(.a(new_n371), .b(new_n338), .c(new_n208), .d(new_n323), .o1(new_n372));
  xnrc02aa1n02x5               g277(.a(\b[30] ), .b(\a[31] ), .out0(new_n373));
  inv000aa1d42x5               g278(.a(new_n373), .o1(new_n374));
  inv000aa1d42x5               g279(.a(new_n371), .o1(new_n375));
  aoi112aa1n02x5               g280(.a(new_n362), .b(new_n373), .c(new_n360), .d(new_n364), .o1(new_n376));
  aoai13aa1n03x5               g281(.a(new_n376), .b(new_n375), .c(new_n324), .d(new_n326), .o1(new_n377));
  aoi012aa1n02x5               g282(.a(new_n362), .b(new_n360), .c(new_n363), .o1(new_n378));
  aoai13aa1n03x5               g283(.a(new_n377), .b(new_n374), .c(new_n372), .d(new_n378), .o1(\s[31] ));
  norp02aa1n02x5               g284(.a(\b[1] ), .b(\a[2] ), .o1(new_n380));
  and002aa1n02x5               g285(.a(\b[1] ), .b(\a[2] ), .o(new_n381));
  aoi112aa1n02x5               g286(.a(new_n381), .b(new_n380), .c(\a[1] ), .d(\b[0] ), .o1(new_n382));
  inv000aa1d42x5               g287(.a(new_n101), .o1(new_n383));
  oai112aa1n02x5               g288(.a(new_n383), .b(new_n102), .c(\b[1] ), .d(\a[2] ), .o1(new_n384));
  obai22aa1n02x7               g289(.a(new_n98), .b(new_n128), .c(new_n382), .d(new_n384), .out0(\s[3] ));
  nanp02aa1n02x5               g290(.a(new_n98), .b(new_n128), .o1(new_n386));
  xnbna2aa1n03x5               g291(.a(new_n127), .b(new_n386), .c(new_n383), .out0(\s[4] ));
  nanp02aa1n02x5               g292(.a(new_n129), .b(new_n104), .o1(new_n388));
  xnrc02aa1n02x5               g293(.a(\b[4] ), .b(\a[5] ), .out0(new_n389));
  aoi112aa1n02x5               g294(.a(new_n389), .b(new_n99), .c(new_n100), .d(new_n101), .o1(new_n390));
  ao0022aa1n03x5               g295(.a(new_n388), .b(new_n389), .c(new_n390), .d(new_n129), .o(\s[5] ));
  nanp02aa1n02x5               g296(.a(\b[5] ), .b(\a[6] ), .o1(new_n392));
  nanb02aa1n02x5               g297(.a(new_n113), .b(new_n392), .out0(new_n393));
  nanb02aa1n02x5               g298(.a(new_n389), .b(new_n388), .out0(new_n394));
  xobna2aa1n03x5               g299(.a(new_n393), .b(new_n394), .c(new_n119), .out0(\s[6] ));
  norb02aa1n02x5               g300(.a(new_n109), .b(new_n108), .out0(new_n396));
  inv000aa1d42x5               g301(.a(new_n396), .o1(new_n397));
  nona22aa1n02x4               g302(.a(new_n388), .b(new_n389), .c(new_n393), .out0(new_n398));
  aoi112aa1n02x5               g303(.a(new_n397), .b(new_n113), .c(new_n392), .d(new_n111), .o1(new_n399));
  nanb02aa1n02x5               g304(.a(new_n120), .b(new_n398), .out0(new_n400));
  ao0022aa1n03x5               g305(.a(new_n400), .b(new_n397), .c(new_n398), .d(new_n399), .o(\s[7] ));
  norb02aa1n02x5               g306(.a(new_n107), .b(new_n106), .out0(new_n402));
  nanp02aa1n02x5               g307(.a(new_n400), .b(new_n396), .o1(new_n403));
  xnbna2aa1n03x5               g308(.a(new_n402), .b(new_n403), .c(new_n121), .out0(\s[8] ));
  aoi112aa1n02x5               g309(.a(new_n124), .b(new_n106), .c(new_n107), .d(new_n108), .o1(new_n405));
  aobi12aa1n02x5               g310(.a(new_n405), .b(new_n120), .c(new_n118), .out0(new_n406));
  ao0022aa1n03x5               g311(.a(new_n131), .b(new_n124), .c(new_n406), .d(new_n117), .o(\s[9] ));
endmodule


