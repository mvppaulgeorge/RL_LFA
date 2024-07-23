// Benchmark "adder" written by ABC on Wed Jul 17 19:23:55 2024

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
    new_n139, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n172, new_n173, new_n174, new_n175, new_n176,
    new_n177, new_n178, new_n179, new_n180, new_n182, new_n183, new_n184,
    new_n185, new_n186, new_n188, new_n189, new_n190, new_n191, new_n192,
    new_n193, new_n194, new_n195, new_n197, new_n198, new_n199, new_n200,
    new_n201, new_n202, new_n203, new_n204, new_n205, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n228, new_n229, new_n230,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n240,
    new_n241, new_n242, new_n243, new_n244, new_n245, new_n246, new_n247,
    new_n248, new_n249, new_n250, new_n251, new_n252, new_n253, new_n254,
    new_n256, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n283, new_n284, new_n285,
    new_n286, new_n287, new_n288, new_n289, new_n290, new_n291, new_n292,
    new_n293, new_n294, new_n295, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n320, new_n321, new_n322, new_n323,
    new_n324, new_n325, new_n326, new_n328, new_n329, new_n330, new_n331,
    new_n332, new_n333, new_n334, new_n335, new_n336, new_n337, new_n338,
    new_n339, new_n341, new_n342, new_n343, new_n344, new_n345, new_n346,
    new_n347, new_n349, new_n350, new_n351, new_n352, new_n353, new_n354,
    new_n355, new_n358, new_n359, new_n360, new_n361, new_n362, new_n363,
    new_n364, new_n365, new_n366, new_n367, new_n369, new_n370, new_n371,
    new_n372, new_n373, new_n374, new_n375, new_n376, new_n377, new_n378,
    new_n379, new_n382, new_n383, new_n385, new_n386, new_n387, new_n388,
    new_n390, new_n391, new_n393, new_n394, new_n395, new_n397, new_n398,
    new_n400;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1n02x5               g002(.a(new_n97), .o1(new_n98));
  orn002aa1n24x5               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nand02aa1d04x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand02aa1d04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nand02aa1d06x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nor002aa1n16x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanb02aa1n03x5               g009(.a(new_n103), .b(new_n104), .out0(new_n105));
  nor002aa1n06x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nand42aa1d28x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  norb03aa1n03x5               g012(.a(new_n107), .b(new_n103), .c(new_n106), .out0(new_n108));
  aoai13aa1n06x5               g013(.a(new_n108), .b(new_n105), .c(new_n99), .d(new_n102), .o1(new_n109));
  nor042aa1d18x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  inv040aa1n06x5               g015(.a(new_n110), .o1(new_n111));
  oai122aa1n02x7               g016(.a(new_n111), .b(\a[7] ), .c(\b[6] ), .d(\a[5] ), .e(\b[4] ), .o1(new_n112));
  oai012aa1n12x5               g017(.a(new_n107), .b(\b[7] ), .c(\a[8] ), .o1(new_n113));
  aoi022aa1d24x5               g018(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n114));
  aoi022aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  nor043aa1n03x5               g021(.a(new_n112), .b(new_n116), .c(new_n113), .o1(new_n117));
  nanp02aa1n03x5               g022(.a(new_n117), .b(new_n109), .o1(new_n118));
  nand02aa1d16x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  oai022aa1n02x5               g024(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n120));
  nanp02aa1n24x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  oai112aa1n06x5               g026(.a(new_n111), .b(new_n121), .c(\b[4] ), .d(\a[5] ), .o1(new_n122));
  aoai13aa1n09x5               g027(.a(new_n119), .b(new_n120), .c(new_n122), .d(new_n114), .o1(new_n123));
  tech160nm_fixnrc02aa1n05x5   g028(.a(\b[8] ), .b(\a[9] ), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n98), .b(new_n124), .c(new_n118), .d(new_n123), .o1(new_n125));
  xorc02aa1n12x5               g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  nand42aa1n04x5               g031(.a(new_n102), .b(new_n99), .o1(new_n127));
  norb02aa1n06x5               g032(.a(new_n104), .b(new_n103), .out0(new_n128));
  nona22aa1n03x5               g033(.a(new_n107), .b(new_n106), .c(new_n103), .out0(new_n129));
  aoi012aa1n12x5               g034(.a(new_n129), .b(new_n127), .c(new_n128), .o1(new_n130));
  nor002aa1d32x5               g035(.a(\b[6] ), .b(\a[7] ), .o1(new_n131));
  nor002aa1d24x5               g036(.a(\b[4] ), .b(\a[5] ), .o1(new_n132));
  nor043aa1n03x5               g037(.a(new_n110), .b(new_n132), .c(new_n131), .o1(new_n133));
  nanp02aa1n04x5               g038(.a(\b[4] ), .b(\a[5] ), .o1(new_n134));
  nand02aa1n02x5               g039(.a(new_n134), .b(new_n119), .o1(new_n135));
  nona23aa1n09x5               g040(.a(new_n133), .b(new_n114), .c(new_n135), .d(new_n113), .out0(new_n136));
  oai012aa1n18x5               g041(.a(new_n123), .b(new_n130), .c(new_n136), .o1(new_n137));
  xorc02aa1n12x5               g042(.a(\a[9] ), .b(\b[8] ), .out0(new_n138));
  aoi112aa1n02x5               g043(.a(new_n126), .b(new_n97), .c(new_n137), .d(new_n138), .o1(new_n139));
  aoi012aa1n02x5               g044(.a(new_n139), .b(new_n125), .c(new_n126), .o1(\s[10] ));
  aoai13aa1n06x5               g045(.a(new_n126), .b(new_n97), .c(new_n137), .d(new_n138), .o1(new_n141));
  oaoi03aa1n02x5               g046(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n142));
  inv000aa1n02x5               g047(.a(new_n142), .o1(new_n143));
  nor002aa1d32x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  nand42aa1n20x5               g049(.a(\b[10] ), .b(\a[11] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n146), .b(new_n141), .c(new_n143), .out0(\s[11] ));
  aoai13aa1n03x5               g052(.a(new_n146), .b(new_n142), .c(new_n125), .d(new_n126), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n144), .o1(new_n149));
  inv000aa1n02x5               g054(.a(new_n146), .o1(new_n150));
  aoai13aa1n02x5               g055(.a(new_n149), .b(new_n150), .c(new_n141), .d(new_n143), .o1(new_n151));
  nor002aa1d32x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nand02aa1d28x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  inv000aa1d42x5               g059(.a(new_n152), .o1(new_n155));
  aoi012aa1n02x5               g060(.a(new_n144), .b(new_n155), .c(new_n153), .o1(new_n156));
  aoi022aa1n02x5               g061(.a(new_n151), .b(new_n154), .c(new_n148), .d(new_n156), .o1(\s[12] ));
  norp02aa1n06x5               g062(.a(\b[7] ), .b(\a[8] ), .o1(new_n158));
  oai012aa1n02x5               g063(.a(new_n119), .b(new_n158), .c(new_n131), .o1(new_n159));
  norb03aa1n03x5               g064(.a(new_n121), .b(new_n132), .c(new_n110), .out0(new_n160));
  nona23aa1n09x5               g065(.a(new_n114), .b(new_n119), .c(new_n158), .d(new_n131), .out0(new_n161));
  oai012aa1n06x5               g066(.a(new_n159), .b(new_n161), .c(new_n160), .o1(new_n162));
  xnrc02aa1n02x5               g067(.a(\b[9] ), .b(\a[10] ), .out0(new_n163));
  nona23aa1n09x5               g068(.a(new_n153), .b(new_n145), .c(new_n144), .d(new_n152), .out0(new_n164));
  nor043aa1n02x5               g069(.a(new_n164), .b(new_n163), .c(new_n124), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n162), .c(new_n117), .d(new_n109), .o1(new_n166));
  nano23aa1n03x7               g071(.a(new_n144), .b(new_n152), .c(new_n153), .d(new_n145), .out0(new_n167));
  nand03aa1n02x5               g072(.a(new_n167), .b(new_n138), .c(new_n126), .o1(new_n168));
  nanp02aa1n06x5               g073(.a(\b[9] ), .b(\a[10] ), .o1(new_n169));
  oaih22aa1n04x5               g074(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n170));
  nanb03aa1n02x5               g075(.a(new_n152), .b(new_n153), .c(new_n145), .out0(new_n171));
  nano32aa1n03x7               g076(.a(new_n171), .b(new_n170), .c(new_n149), .d(new_n169), .out0(new_n172));
  aoi012aa1n09x5               g077(.a(new_n152), .b(new_n144), .c(new_n153), .o1(new_n173));
  inv000aa1n02x5               g078(.a(new_n173), .o1(new_n174));
  nor042aa1n03x5               g079(.a(new_n172), .b(new_n174), .o1(new_n175));
  aoai13aa1n06x5               g080(.a(new_n175), .b(new_n168), .c(new_n118), .d(new_n123), .o1(new_n176));
  nor002aa1n10x5               g081(.a(\b[12] ), .b(\a[13] ), .o1(new_n177));
  nand42aa1d28x5               g082(.a(\b[12] ), .b(\a[13] ), .o1(new_n178));
  norb02aa1n06x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  norp03aa1n02x5               g084(.a(new_n172), .b(new_n174), .c(new_n179), .o1(new_n180));
  aoi022aa1n02x5               g085(.a(new_n176), .b(new_n179), .c(new_n166), .d(new_n180), .o1(\s[13] ));
  nor042aa1n09x5               g086(.a(\b[13] ), .b(\a[14] ), .o1(new_n182));
  nand42aa1d28x5               g087(.a(\b[13] ), .b(\a[14] ), .o1(new_n183));
  norb02aa1n03x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  aoai13aa1n02x7               g089(.a(new_n184), .b(new_n177), .c(new_n176), .d(new_n178), .o1(new_n185));
  aoi112aa1n02x7               g090(.a(new_n177), .b(new_n184), .c(new_n176), .d(new_n179), .o1(new_n186));
  norb02aa1n03x4               g091(.a(new_n185), .b(new_n186), .out0(\s[14] ));
  inv040aa1n03x5               g092(.a(new_n175), .o1(new_n188));
  nano23aa1n06x5               g093(.a(new_n177), .b(new_n182), .c(new_n183), .d(new_n178), .out0(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n188), .c(new_n137), .d(new_n165), .o1(new_n190));
  oa0012aa1n06x5               g095(.a(new_n183), .b(new_n182), .c(new_n177), .o(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  nor002aa1d32x5               g097(.a(\b[14] ), .b(\a[15] ), .o1(new_n193));
  nand02aa1d24x5               g098(.a(\b[14] ), .b(\a[15] ), .o1(new_n194));
  norb02aa1n09x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  xnbna2aa1n03x5               g100(.a(new_n195), .b(new_n190), .c(new_n192), .out0(\s[15] ));
  aoai13aa1n03x5               g101(.a(new_n195), .b(new_n191), .c(new_n176), .d(new_n189), .o1(new_n197));
  inv000aa1d42x5               g102(.a(new_n193), .o1(new_n198));
  inv040aa1n02x5               g103(.a(new_n195), .o1(new_n199));
  aoai13aa1n02x5               g104(.a(new_n198), .b(new_n199), .c(new_n190), .d(new_n192), .o1(new_n200));
  nor002aa1d24x5               g105(.a(\b[15] ), .b(\a[16] ), .o1(new_n201));
  nand02aa1n16x5               g106(.a(\b[15] ), .b(\a[16] ), .o1(new_n202));
  norb02aa1n06x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n201), .o1(new_n204));
  aoi012aa1n02x5               g109(.a(new_n193), .b(new_n204), .c(new_n202), .o1(new_n205));
  aoi022aa1n03x5               g110(.a(new_n200), .b(new_n203), .c(new_n197), .d(new_n205), .o1(\s[16] ));
  nand23aa1n04x5               g111(.a(new_n189), .b(new_n195), .c(new_n203), .o1(new_n207));
  nor042aa1n04x5               g112(.a(new_n207), .b(new_n168), .o1(new_n208));
  inv000aa1n02x5               g113(.a(new_n169), .o1(new_n209));
  aoi012aa1n02x7               g114(.a(new_n144), .b(\a[10] ), .c(\b[9] ), .o1(new_n210));
  nano22aa1n03x7               g115(.a(new_n152), .b(new_n145), .c(new_n153), .out0(new_n211));
  oai112aa1n04x5               g116(.a(new_n211), .b(new_n210), .c(new_n170), .d(new_n209), .o1(new_n212));
  oai022aa1d18x5               g117(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n213));
  nano22aa1n03x7               g118(.a(new_n201), .b(new_n194), .c(new_n202), .out0(new_n214));
  aoi012aa1n03x5               g119(.a(new_n193), .b(\a[14] ), .c(\b[13] ), .o1(new_n215));
  aoi012aa1n06x5               g120(.a(new_n201), .b(new_n193), .c(new_n202), .o1(new_n216));
  inv020aa1n02x5               g121(.a(new_n216), .o1(new_n217));
  aoi013aa1n06x4               g122(.a(new_n217), .b(new_n214), .c(new_n215), .d(new_n213), .o1(new_n218));
  aoai13aa1n12x5               g123(.a(new_n218), .b(new_n207), .c(new_n212), .d(new_n173), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[17] ), .b(\b[16] ), .out0(new_n220));
  aoai13aa1n06x5               g125(.a(new_n220), .b(new_n219), .c(new_n137), .d(new_n208), .o1(new_n221));
  aoi012aa1n06x5               g126(.a(new_n162), .b(new_n117), .c(new_n109), .o1(new_n222));
  nano32aa1n03x7               g127(.a(new_n199), .b(new_n203), .c(new_n179), .d(new_n184), .out0(new_n223));
  nanp02aa1n03x5               g128(.a(new_n223), .b(new_n165), .o1(new_n224));
  aoi113aa1n02x5               g129(.a(new_n220), .b(new_n217), .c(new_n214), .d(new_n215), .e(new_n213), .o1(new_n225));
  oai122aa1n02x7               g130(.a(new_n225), .b(new_n222), .c(new_n224), .d(new_n207), .e(new_n175), .o1(new_n226));
  and002aa1n02x5               g131(.a(new_n226), .b(new_n221), .o(\s[17] ));
  inv000aa1d42x5               g132(.a(\a[17] ), .o1(new_n228));
  nanb02aa1n02x5               g133(.a(\b[16] ), .b(new_n228), .out0(new_n229));
  xorc02aa1n02x5               g134(.a(\a[18] ), .b(\b[17] ), .out0(new_n230));
  xnbna2aa1n03x5               g135(.a(new_n230), .b(new_n221), .c(new_n229), .out0(\s[18] ));
  inv020aa1n04x5               g136(.a(\a[18] ), .o1(new_n232));
  xroi22aa1d06x4               g137(.a(new_n228), .b(\b[16] ), .c(new_n232), .d(\b[17] ), .out0(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n219), .c(new_n137), .d(new_n208), .o1(new_n234));
  oaoi03aa1n12x5               g139(.a(\a[18] ), .b(\b[17] ), .c(new_n229), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  xorc02aa1n02x5               g141(.a(\a[19] ), .b(\b[18] ), .out0(new_n237));
  xnbna2aa1n03x5               g142(.a(new_n237), .b(new_n234), .c(new_n236), .out0(\s[19] ));
  xnrc02aa1n02x5               g143(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanb03aa1n03x5               g144(.a(new_n201), .b(new_n202), .c(new_n194), .out0(new_n240));
  oai112aa1n02x5               g145(.a(new_n198), .b(new_n183), .c(new_n182), .d(new_n177), .o1(new_n241));
  oai012aa1n02x5               g146(.a(new_n216), .b(new_n241), .c(new_n240), .o1(new_n242));
  oaoi13aa1n09x5               g147(.a(new_n242), .b(new_n223), .c(new_n172), .d(new_n174), .o1(new_n243));
  oai012aa1n12x5               g148(.a(new_n243), .b(new_n222), .c(new_n224), .o1(new_n244));
  aoai13aa1n03x5               g149(.a(new_n237), .b(new_n235), .c(new_n244), .d(new_n233), .o1(new_n245));
  inv000aa1d42x5               g150(.a(\a[19] ), .o1(new_n246));
  inv000aa1d42x5               g151(.a(\b[18] ), .o1(new_n247));
  nanp02aa1n02x5               g152(.a(new_n247), .b(new_n246), .o1(new_n248));
  inv000aa1n02x5               g153(.a(new_n237), .o1(new_n249));
  aoai13aa1n02x7               g154(.a(new_n248), .b(new_n249), .c(new_n234), .d(new_n236), .o1(new_n250));
  nor002aa1d24x5               g155(.a(\b[19] ), .b(\a[20] ), .o1(new_n251));
  nand02aa1d28x5               g156(.a(\b[19] ), .b(\a[20] ), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n252), .b(new_n251), .out0(new_n253));
  aboi22aa1n03x5               g158(.a(new_n251), .b(new_n252), .c(new_n246), .d(new_n247), .out0(new_n254));
  aoi022aa1n03x5               g159(.a(new_n250), .b(new_n253), .c(new_n245), .d(new_n254), .o1(\s[20] ));
  nano32aa1n02x5               g160(.a(new_n249), .b(new_n230), .c(new_n220), .d(new_n253), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n219), .c(new_n137), .d(new_n208), .o1(new_n257));
  oai022aa1d24x5               g162(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n258));
  inv000aa1n02x5               g163(.a(new_n252), .o1(new_n259));
  aoi012aa1n06x5               g164(.a(new_n251), .b(\a[19] ), .c(\b[18] ), .o1(new_n260));
  nand42aa1d28x5               g165(.a(\b[17] ), .b(\a[18] ), .o1(new_n261));
  oai012aa1d24x5               g166(.a(new_n261), .b(\b[18] ), .c(\a[19] ), .o1(new_n262));
  nona23aa1d18x5               g167(.a(new_n260), .b(new_n258), .c(new_n262), .d(new_n259), .out0(new_n263));
  aoi013aa1n09x5               g168(.a(new_n251), .b(new_n252), .c(new_n246), .d(new_n247), .o1(new_n264));
  nand02aa1d06x5               g169(.a(new_n263), .b(new_n264), .o1(new_n265));
  nor042aa1d18x5               g170(.a(\b[20] ), .b(\a[21] ), .o1(new_n266));
  nand42aa1d28x5               g171(.a(\b[20] ), .b(\a[21] ), .o1(new_n267));
  norb02aa1n06x4               g172(.a(new_n267), .b(new_n266), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n265), .c(new_n244), .d(new_n256), .o1(new_n269));
  oai022aa1n02x5               g174(.a(new_n246), .b(new_n247), .c(\b[19] ), .d(\a[20] ), .o1(new_n270));
  nano23aa1n06x5               g175(.a(new_n262), .b(new_n270), .c(new_n258), .d(new_n252), .out0(new_n271));
  inv030aa1n02x5               g176(.a(new_n268), .o1(new_n272));
  nano22aa1n02x4               g177(.a(new_n271), .b(new_n264), .c(new_n272), .out0(new_n273));
  aobi12aa1n03x7               g178(.a(new_n269), .b(new_n273), .c(new_n257), .out0(\s[21] ));
  inv000aa1d42x5               g179(.a(new_n265), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n266), .o1(new_n276));
  aoai13aa1n02x7               g181(.a(new_n276), .b(new_n272), .c(new_n257), .d(new_n275), .o1(new_n277));
  nor042aa1n06x5               g182(.a(\b[21] ), .b(\a[22] ), .o1(new_n278));
  nand42aa1d28x5               g183(.a(\b[21] ), .b(\a[22] ), .o1(new_n279));
  norb02aa1n02x7               g184(.a(new_n279), .b(new_n278), .out0(new_n280));
  aoib12aa1n02x5               g185(.a(new_n266), .b(new_n279), .c(new_n278), .out0(new_n281));
  aoi022aa1n02x7               g186(.a(new_n277), .b(new_n280), .c(new_n269), .d(new_n281), .o1(\s[22] ));
  inv000aa1d42x5               g187(.a(\a[20] ), .o1(new_n283));
  xroi22aa1d04x5               g188(.a(new_n246), .b(\b[18] ), .c(new_n283), .d(\b[19] ), .out0(new_n284));
  nano23aa1d15x5               g189(.a(new_n266), .b(new_n278), .c(new_n279), .d(new_n267), .out0(new_n285));
  and003aa1n02x5               g190(.a(new_n284), .b(new_n233), .c(new_n285), .o(new_n286));
  aoai13aa1n06x5               g191(.a(new_n286), .b(new_n219), .c(new_n137), .d(new_n208), .o1(new_n287));
  oai022aa1d18x5               g192(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n288));
  aoi022aa1n03x5               g193(.a(new_n265), .b(new_n285), .c(new_n279), .d(new_n288), .o1(new_n289));
  inv040aa1n03x5               g194(.a(new_n289), .o1(new_n290));
  nor002aa1d32x5               g195(.a(\b[22] ), .b(\a[23] ), .o1(new_n291));
  nand02aa1d28x5               g196(.a(\b[22] ), .b(\a[23] ), .o1(new_n292));
  norb02aa1n15x5               g197(.a(new_n292), .b(new_n291), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n290), .c(new_n244), .d(new_n286), .o1(new_n294));
  aoi122aa1n02x5               g199(.a(new_n293), .b(new_n279), .c(new_n288), .d(new_n265), .e(new_n285), .o1(new_n295));
  aobi12aa1n02x7               g200(.a(new_n294), .b(new_n295), .c(new_n287), .out0(\s[23] ));
  inv040aa1n04x5               g201(.a(new_n291), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n293), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n297), .b(new_n298), .c(new_n287), .d(new_n289), .o1(new_n299));
  nor042aa1d18x5               g204(.a(\b[23] ), .b(\a[24] ), .o1(new_n300));
  nand22aa1n12x5               g205(.a(\b[23] ), .b(\a[24] ), .o1(new_n301));
  norb02aa1n03x5               g206(.a(new_n301), .b(new_n300), .out0(new_n302));
  aoib12aa1n02x5               g207(.a(new_n291), .b(new_n301), .c(new_n300), .out0(new_n303));
  aoi022aa1n03x5               g208(.a(new_n299), .b(new_n302), .c(new_n294), .d(new_n303), .o1(\s[24] ));
  nand23aa1n06x5               g209(.a(new_n285), .b(new_n293), .c(new_n302), .o1(new_n305));
  nano22aa1n02x4               g210(.a(new_n305), .b(new_n233), .c(new_n284), .out0(new_n306));
  nano22aa1d18x5               g211(.a(new_n300), .b(new_n292), .c(new_n301), .out0(new_n307));
  aoi012aa1n12x5               g212(.a(new_n291), .b(\a[22] ), .c(\b[21] ), .o1(new_n308));
  oaoi03aa1n12x5               g213(.a(\a[24] ), .b(\b[23] ), .c(new_n297), .o1(new_n309));
  aoi013aa1n09x5               g214(.a(new_n309), .b(new_n307), .c(new_n308), .d(new_n288), .o1(new_n310));
  aoai13aa1n12x5               g215(.a(new_n310), .b(new_n305), .c(new_n263), .d(new_n264), .o1(new_n311));
  xorc02aa1n12x5               g216(.a(\a[25] ), .b(\b[24] ), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n311), .c(new_n244), .d(new_n306), .o1(new_n313));
  aoai13aa1n04x5               g218(.a(new_n306), .b(new_n219), .c(new_n137), .d(new_n208), .o1(new_n314));
  nano32aa1n03x7               g219(.a(new_n272), .b(new_n302), .c(new_n280), .d(new_n293), .out0(new_n315));
  oaib12aa1n06x5               g220(.a(new_n315), .b(new_n271), .c(new_n264), .out0(new_n316));
  aoi113aa1n02x5               g221(.a(new_n312), .b(new_n309), .c(new_n307), .d(new_n308), .e(new_n288), .o1(new_n317));
  and003aa1n02x5               g222(.a(new_n314), .b(new_n317), .c(new_n316), .o(new_n318));
  norb02aa1n03x4               g223(.a(new_n313), .b(new_n318), .out0(\s[25] ));
  inv000aa1d42x5               g224(.a(new_n311), .o1(new_n320));
  nor002aa1d32x5               g225(.a(\b[24] ), .b(\a[25] ), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n321), .o1(new_n322));
  inv000aa1d42x5               g227(.a(new_n312), .o1(new_n323));
  aoai13aa1n02x7               g228(.a(new_n322), .b(new_n323), .c(new_n314), .d(new_n320), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[26] ), .b(\b[25] ), .out0(new_n325));
  norp02aa1n02x5               g230(.a(new_n325), .b(new_n321), .o1(new_n326));
  aoi022aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n313), .d(new_n326), .o1(\s[26] ));
  nand42aa1n03x5               g232(.a(\b[24] ), .b(\a[25] ), .o1(new_n328));
  xnrc02aa1n12x5               g233(.a(\b[25] ), .b(\a[26] ), .out0(new_n329));
  nano22aa1n12x5               g234(.a(new_n329), .b(new_n322), .c(new_n328), .out0(new_n330));
  nano32aa1n03x7               g235(.a(new_n305), .b(new_n284), .c(new_n233), .d(new_n330), .out0(new_n331));
  aoai13aa1n06x5               g236(.a(new_n331), .b(new_n219), .c(new_n137), .d(new_n208), .o1(new_n332));
  inv000aa1d42x5               g237(.a(new_n330), .o1(new_n333));
  oao003aa1n02x5               g238(.a(\a[26] ), .b(\b[25] ), .c(new_n322), .carry(new_n334));
  aoai13aa1n06x5               g239(.a(new_n334), .b(new_n333), .c(new_n316), .d(new_n310), .o1(new_n335));
  xorc02aa1n12x5               g240(.a(\a[27] ), .b(\b[26] ), .out0(new_n336));
  aoai13aa1n06x5               g241(.a(new_n336), .b(new_n335), .c(new_n244), .d(new_n331), .o1(new_n337));
  inv000aa1d42x5               g242(.a(new_n334), .o1(new_n338));
  aoi112aa1n02x5               g243(.a(new_n336), .b(new_n338), .c(new_n311), .d(new_n330), .o1(new_n339));
  aobi12aa1n02x7               g244(.a(new_n337), .b(new_n339), .c(new_n332), .out0(\s[27] ));
  aoi012aa1n09x5               g245(.a(new_n338), .b(new_n311), .c(new_n330), .o1(new_n341));
  norp02aa1n02x5               g246(.a(\b[26] ), .b(\a[27] ), .o1(new_n342));
  inv000aa1n03x5               g247(.a(new_n342), .o1(new_n343));
  inv000aa1d42x5               g248(.a(new_n336), .o1(new_n344));
  aoai13aa1n03x5               g249(.a(new_n343), .b(new_n344), .c(new_n332), .d(new_n341), .o1(new_n345));
  tech160nm_fixorc02aa1n02p5x5 g250(.a(\a[28] ), .b(\b[27] ), .out0(new_n346));
  norp02aa1n02x5               g251(.a(new_n346), .b(new_n342), .o1(new_n347));
  aoi022aa1n03x5               g252(.a(new_n345), .b(new_n346), .c(new_n337), .d(new_n347), .o1(\s[28] ));
  and002aa1n02x5               g253(.a(new_n346), .b(new_n336), .o(new_n349));
  aoai13aa1n02x5               g254(.a(new_n349), .b(new_n335), .c(new_n244), .d(new_n331), .o1(new_n350));
  inv000aa1d42x5               g255(.a(new_n349), .o1(new_n351));
  oao003aa1n02x5               g256(.a(\a[28] ), .b(\b[27] ), .c(new_n343), .carry(new_n352));
  aoai13aa1n03x5               g257(.a(new_n352), .b(new_n351), .c(new_n332), .d(new_n341), .o1(new_n353));
  xorc02aa1n02x5               g258(.a(\a[29] ), .b(\b[28] ), .out0(new_n354));
  norb02aa1n02x5               g259(.a(new_n352), .b(new_n354), .out0(new_n355));
  aoi022aa1n03x5               g260(.a(new_n353), .b(new_n354), .c(new_n350), .d(new_n355), .o1(\s[29] ));
  xorb03aa1n02x5               g261(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g262(.a(new_n344), .b(new_n346), .c(new_n354), .out0(new_n358));
  aoai13aa1n02x5               g263(.a(new_n358), .b(new_n335), .c(new_n244), .d(new_n331), .o1(new_n359));
  inv000aa1n02x5               g264(.a(new_n358), .o1(new_n360));
  inv000aa1d42x5               g265(.a(\b[28] ), .o1(new_n361));
  inv000aa1d42x5               g266(.a(\a[29] ), .o1(new_n362));
  oaib12aa1n02x5               g267(.a(new_n352), .b(\b[28] ), .c(new_n362), .out0(new_n363));
  oaib12aa1n02x5               g268(.a(new_n363), .b(new_n361), .c(\a[29] ), .out0(new_n364));
  aoai13aa1n03x5               g269(.a(new_n364), .b(new_n360), .c(new_n332), .d(new_n341), .o1(new_n365));
  xorc02aa1n02x5               g270(.a(\a[30] ), .b(\b[29] ), .out0(new_n366));
  oaoi13aa1n02x5               g271(.a(new_n366), .b(new_n363), .c(new_n362), .d(new_n361), .o1(new_n367));
  aoi022aa1n03x5               g272(.a(new_n365), .b(new_n366), .c(new_n359), .d(new_n367), .o1(\s[30] ));
  nano32aa1n03x7               g273(.a(new_n344), .b(new_n366), .c(new_n346), .d(new_n354), .out0(new_n369));
  aoai13aa1n02x5               g274(.a(new_n369), .b(new_n335), .c(new_n244), .d(new_n331), .o1(new_n370));
  aoi022aa1n02x5               g275(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n371));
  norb02aa1n02x5               g276(.a(\b[30] ), .b(\a[31] ), .out0(new_n372));
  obai22aa1n02x7               g277(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n373));
  aoi112aa1n02x5               g278(.a(new_n373), .b(new_n372), .c(new_n363), .d(new_n371), .o1(new_n374));
  inv000aa1d42x5               g279(.a(new_n369), .o1(new_n375));
  norp02aa1n02x5               g280(.a(\b[29] ), .b(\a[30] ), .o1(new_n376));
  aoi012aa1n02x5               g281(.a(new_n376), .b(new_n363), .c(new_n371), .o1(new_n377));
  aoai13aa1n03x5               g282(.a(new_n377), .b(new_n375), .c(new_n332), .d(new_n341), .o1(new_n378));
  xorc02aa1n02x5               g283(.a(\a[31] ), .b(\b[30] ), .out0(new_n379));
  aoi022aa1n03x5               g284(.a(new_n378), .b(new_n379), .c(new_n374), .d(new_n370), .o1(\s[31] ));
  xnbna2aa1n03x5               g285(.a(new_n128), .b(new_n102), .c(new_n99), .out0(\s[3] ));
  norb02aa1n02x5               g286(.a(new_n107), .b(new_n106), .out0(new_n382));
  aoi012aa1n02x5               g287(.a(new_n103), .b(new_n127), .c(new_n128), .o1(new_n383));
  oai012aa1n02x5               g288(.a(new_n109), .b(new_n383), .c(new_n382), .o1(\s[4] ));
  inv000aa1d42x5               g289(.a(new_n132), .o1(new_n385));
  aoi022aa1n02x5               g290(.a(new_n109), .b(new_n107), .c(new_n134), .d(new_n385), .o1(new_n386));
  nano22aa1n02x4               g291(.a(new_n132), .b(new_n107), .c(new_n134), .out0(new_n387));
  aoai13aa1n06x5               g292(.a(new_n387), .b(new_n129), .c(new_n127), .d(new_n128), .o1(new_n388));
  norb02aa1n02x5               g293(.a(new_n388), .b(new_n386), .out0(\s[5] ));
  norb02aa1n02x5               g294(.a(new_n121), .b(new_n110), .out0(new_n390));
  nanp02aa1n02x5               g295(.a(new_n388), .b(new_n160), .o1(new_n391));
  aoai13aa1n02x5               g296(.a(new_n391), .b(new_n390), .c(new_n385), .d(new_n388), .o1(\s[6] ));
  inv000aa1d42x5               g297(.a(new_n131), .o1(new_n393));
  nanp02aa1n02x5               g298(.a(\b[6] ), .b(\a[7] ), .o1(new_n394));
  aoi022aa1n02x5               g299(.a(new_n391), .b(new_n121), .c(new_n393), .d(new_n394), .o1(new_n395));
  aoi013aa1n02x4               g300(.a(new_n395), .b(new_n391), .c(new_n114), .d(new_n393), .o1(\s[7] ));
  nanp03aa1n03x5               g301(.a(new_n391), .b(new_n393), .c(new_n114), .o1(new_n397));
  norb02aa1n02x5               g302(.a(new_n119), .b(new_n158), .out0(new_n398));
  xnbna2aa1n03x5               g303(.a(new_n398), .b(new_n397), .c(new_n393), .out0(\s[8] ));
  oai112aa1n02x5               g304(.a(new_n159), .b(new_n124), .c(new_n161), .d(new_n160), .o1(new_n400));
  aboi22aa1n03x5               g305(.a(new_n400), .b(new_n118), .c(new_n137), .d(new_n138), .out0(\s[9] ));
endmodule


