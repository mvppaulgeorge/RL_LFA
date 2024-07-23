// Benchmark "adder" written by ABC on Thu Jul 18 07:38:48 2024

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
    new_n139, new_n140, new_n141, new_n142, new_n143, new_n144, new_n146,
    new_n147, new_n148, new_n149, new_n150, new_n151, new_n152, new_n154,
    new_n155, new_n156, new_n157, new_n158, new_n159, new_n160, new_n161,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n172, new_n173, new_n175, new_n176, new_n177,
    new_n178, new_n179, new_n180, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n187, new_n188, new_n189, new_n190, new_n192, new_n193,
    new_n194, new_n195, new_n196, new_n197, new_n198, new_n199, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n214, new_n215, new_n216,
    new_n217, new_n218, new_n219, new_n220, new_n222, new_n223, new_n224,
    new_n225, new_n226, new_n227, new_n228, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n238, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n260, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n274, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n298, new_n299, new_n300, new_n301, new_n302,
    new_n303, new_n304, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n312, new_n313, new_n314, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n321, new_n322, new_n323, new_n324, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n331, new_n332, new_n333,
    new_n334, new_n337, new_n338, new_n339, new_n340, new_n341, new_n342,
    new_n343, new_n344, new_n345, new_n346, new_n347, new_n349, new_n350,
    new_n351, new_n352, new_n353, new_n354, new_n355, new_n358, new_n359,
    new_n362, new_n363, new_n365, new_n366, new_n368, new_n369, new_n371;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nanp02aa1n03x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanb03aa1n06x5               g005(.a(new_n97), .b(new_n100), .c(new_n99), .out0(new_n101));
  norp02aa1n06x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nand02aa1n03x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor002aa1d24x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n02x5               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  oai012aa1n04x7               g011(.a(new_n103), .b(new_n104), .c(new_n102), .o1(new_n107));
  aoai13aa1n04x5               g012(.a(new_n107), .b(new_n106), .c(new_n98), .d(new_n101), .o1(new_n108));
  nor022aa1n16x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nand42aa1n03x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nand42aa1n10x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nor022aa1n08x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nona23aa1n02x4               g017(.a(new_n111), .b(new_n110), .c(new_n112), .d(new_n109), .out0(new_n113));
  xorc02aa1n12x5               g018(.a(\a[8] ), .b(\b[7] ), .out0(new_n114));
  nor022aa1n08x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nand22aa1n09x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  norb02aa1n03x5               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  nano22aa1n03x7               g022(.a(new_n113), .b(new_n114), .c(new_n117), .out0(new_n118));
  nanp02aa1n02x5               g023(.a(new_n108), .b(new_n118), .o1(new_n119));
  nona22aa1n02x4               g024(.a(new_n111), .b(new_n112), .c(new_n109), .out0(new_n120));
  nanb03aa1n06x5               g025(.a(new_n115), .b(new_n116), .c(new_n111), .out0(new_n121));
  inv000aa1n02x5               g026(.a(new_n121), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\a[8] ), .o1(new_n123));
  inv000aa1d42x5               g028(.a(\b[7] ), .o1(new_n124));
  oao003aa1n06x5               g029(.a(new_n123), .b(new_n124), .c(new_n115), .carry(new_n125));
  aoi013aa1n06x4               g030(.a(new_n125), .b(new_n122), .c(new_n120), .d(new_n114), .o1(new_n126));
  nor022aa1n08x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  tech160nm_finand02aa1n03p5x5 g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  aobi12aa1n02x5               g034(.a(new_n129), .b(new_n119), .c(new_n126), .out0(new_n130));
  aob012aa1n02x5               g035(.a(new_n98), .b(new_n99), .c(new_n100), .out0(new_n131));
  norb02aa1n03x5               g036(.a(new_n103), .b(new_n102), .out0(new_n132));
  norb02aa1n06x4               g037(.a(new_n105), .b(new_n104), .out0(new_n133));
  nand23aa1n04x5               g038(.a(new_n131), .b(new_n132), .c(new_n133), .o1(new_n134));
  nanb02aa1n03x5               g039(.a(new_n109), .b(new_n110), .out0(new_n135));
  norb02aa1n02x7               g040(.a(new_n111), .b(new_n112), .out0(new_n136));
  tech160nm_fixnrc02aa1n04x5   g041(.a(\b[7] ), .b(\a[8] ), .out0(new_n137));
  nona23aa1n09x5               g042(.a(new_n136), .b(new_n117), .c(new_n137), .d(new_n135), .out0(new_n138));
  aoai13aa1n12x5               g043(.a(new_n126), .b(new_n138), .c(new_n134), .d(new_n107), .o1(new_n139));
  aoi012aa1n02x5               g044(.a(new_n127), .b(new_n139), .c(new_n128), .o1(new_n140));
  nor022aa1n08x5               g045(.a(\b[9] ), .b(\a[10] ), .o1(new_n141));
  nand42aa1n04x5               g046(.a(\b[9] ), .b(\a[10] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  nona22aa1n06x5               g048(.a(new_n142), .b(new_n141), .c(new_n127), .out0(new_n144));
  oai022aa1n02x5               g049(.a(new_n140), .b(new_n143), .c(new_n144), .d(new_n130), .o1(\s[10] ));
  norb03aa1n02x7               g050(.a(new_n111), .b(new_n109), .c(new_n112), .out0(new_n146));
  inv000aa1n02x5               g051(.a(new_n125), .o1(new_n147));
  oai013aa1n03x5               g052(.a(new_n147), .b(new_n146), .c(new_n121), .d(new_n137), .o1(new_n148));
  nano23aa1n06x5               g053(.a(new_n127), .b(new_n141), .c(new_n142), .d(new_n128), .out0(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n148), .c(new_n108), .d(new_n118), .o1(new_n150));
  oai012aa1n02x5               g055(.a(new_n142), .b(new_n141), .c(new_n127), .o1(new_n151));
  tech160nm_fixorc02aa1n03p5x5 g056(.a(\a[11] ), .b(\b[10] ), .out0(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n152), .b(new_n150), .c(new_n151), .out0(\s[11] ));
  norp02aa1n02x5               g058(.a(\b[10] ), .b(\a[11] ), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  aob012aa1n02x5               g060(.a(new_n152), .b(new_n150), .c(new_n151), .out0(new_n156));
  tech160nm_fixorc02aa1n03p5x5 g061(.a(\a[12] ), .b(\b[11] ), .out0(new_n157));
  and002aa1n02x5               g062(.a(\b[11] ), .b(\a[12] ), .o(new_n158));
  oaih22aa1n06x5               g063(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n159));
  nor042aa1n03x5               g064(.a(new_n159), .b(new_n158), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(new_n156), .b(new_n160), .o1(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n157), .c(new_n156), .d(new_n155), .o1(\s[12] ));
  and003aa1n02x5               g067(.a(new_n149), .b(new_n157), .c(new_n152), .o(new_n163));
  aoai13aa1n06x5               g068(.a(new_n163), .b(new_n148), .c(new_n108), .d(new_n118), .o1(new_n164));
  aoi022aa1n12x5               g069(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n165));
  nanp03aa1d12x5               g070(.a(new_n144), .b(new_n160), .c(new_n165), .o1(new_n166));
  aob012aa1n03x5               g071(.a(new_n159), .b(\b[11] ), .c(\a[12] ), .out0(new_n167));
  and002aa1n24x5               g072(.a(new_n166), .b(new_n167), .o(new_n168));
  nand42aa1n02x5               g073(.a(new_n164), .b(new_n168), .o1(new_n169));
  nanp02aa1n09x5               g074(.a(\b[12] ), .b(\a[13] ), .o1(new_n170));
  nor002aa1d32x5               g075(.a(\b[12] ), .b(\a[13] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  nano22aa1n02x4               g077(.a(new_n172), .b(new_n166), .c(new_n167), .out0(new_n173));
  aoi022aa1n02x5               g078(.a(new_n169), .b(new_n172), .c(new_n164), .d(new_n173), .o1(\s[13] ));
  aobi12aa1n02x5               g079(.a(new_n172), .b(new_n164), .c(new_n168), .out0(new_n175));
  aoi012aa1n02x5               g080(.a(new_n171), .b(new_n169), .c(new_n170), .o1(new_n176));
  nor002aa1n08x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  nand02aa1n08x5               g082(.a(\b[13] ), .b(\a[14] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  nona22aa1n03x5               g084(.a(new_n178), .b(new_n177), .c(new_n171), .out0(new_n180));
  oai022aa1n02x5               g085(.a(new_n176), .b(new_n179), .c(new_n180), .d(new_n175), .o1(\s[14] ));
  inv000aa1n02x5               g086(.a(new_n171), .o1(new_n182));
  oaoi03aa1n02x5               g087(.a(\a[14] ), .b(\b[13] ), .c(new_n182), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n168), .o1(new_n185));
  nano23aa1n06x5               g090(.a(new_n177), .b(new_n171), .c(new_n178), .d(new_n170), .out0(new_n186));
  aoai13aa1n06x5               g091(.a(new_n186), .b(new_n185), .c(new_n139), .d(new_n163), .o1(new_n187));
  nand02aa1n03x5               g092(.a(\b[14] ), .b(\a[15] ), .o1(new_n188));
  nor042aa1n12x5               g093(.a(\b[14] ), .b(\a[15] ), .o1(new_n189));
  norb02aa1d21x5               g094(.a(new_n188), .b(new_n189), .out0(new_n190));
  xnbna2aa1n03x5               g095(.a(new_n190), .b(new_n187), .c(new_n184), .out0(\s[15] ));
  inv000aa1d42x5               g096(.a(new_n189), .o1(new_n192));
  aoai13aa1n03x5               g097(.a(new_n190), .b(new_n183), .c(new_n169), .d(new_n186), .o1(new_n193));
  tech160nm_fixorc02aa1n04x5   g098(.a(\a[16] ), .b(\b[15] ), .out0(new_n194));
  inv000aa1d42x5               g099(.a(new_n190), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[15] ), .b(\a[16] ), .o1(new_n196));
  oai022aa1n02x5               g101(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(new_n198));
  aoai13aa1n02x7               g103(.a(new_n198), .b(new_n195), .c(new_n187), .d(new_n184), .o1(new_n199));
  aoai13aa1n02x5               g104(.a(new_n199), .b(new_n194), .c(new_n193), .d(new_n192), .o1(\s[16] ));
  nand23aa1n06x5               g105(.a(new_n186), .b(new_n190), .c(new_n194), .o1(new_n201));
  nano32aa1n09x5               g106(.a(new_n201), .b(new_n157), .c(new_n149), .d(new_n152), .out0(new_n202));
  aoai13aa1n06x5               g107(.a(new_n202), .b(new_n148), .c(new_n108), .d(new_n118), .o1(new_n203));
  aoi022aa1n02x5               g108(.a(\b[14] ), .b(\a[15] ), .c(\a[14] ), .d(\b[13] ), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n196), .b(new_n197), .c(new_n180), .d(new_n204), .o1(new_n205));
  aoai13aa1n12x5               g110(.a(new_n205), .b(new_n201), .c(new_n166), .d(new_n167), .o1(new_n206));
  inv040aa1n06x5               g111(.a(new_n206), .o1(new_n207));
  nanp02aa1n06x5               g112(.a(new_n203), .b(new_n207), .o1(new_n208));
  tech160nm_fixorc02aa1n03p5x5 g113(.a(\a[17] ), .b(\b[16] ), .out0(new_n209));
  aoi012aa1n02x5               g114(.a(new_n201), .b(new_n166), .c(new_n167), .o1(new_n210));
  tech160nm_fiao0012aa1n02p5x5 g115(.a(new_n209), .b(new_n196), .c(new_n197), .o(new_n211));
  aoi113aa1n02x5               g116(.a(new_n210), .b(new_n211), .c(new_n180), .d(new_n198), .e(new_n204), .o1(new_n212));
  aoi022aa1n02x5               g117(.a(new_n208), .b(new_n209), .c(new_n203), .d(new_n212), .o1(\s[17] ));
  inv000aa1d42x5               g118(.a(\a[17] ), .o1(new_n214));
  nanb02aa1n02x5               g119(.a(\b[16] ), .b(new_n214), .out0(new_n215));
  aoai13aa1n02x7               g120(.a(new_n209), .b(new_n206), .c(new_n139), .d(new_n202), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[18] ), .b(\b[17] ), .out0(new_n217));
  and002aa1n06x5               g122(.a(\b[17] ), .b(\a[18] ), .o(new_n218));
  oaih22aa1n06x5               g123(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n219));
  nona22aa1n02x4               g124(.a(new_n216), .b(new_n218), .c(new_n219), .out0(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n217), .c(new_n215), .d(new_n216), .o1(\s[18] ));
  and002aa1n02x5               g126(.a(new_n217), .b(new_n209), .o(new_n222));
  aoai13aa1n06x5               g127(.a(new_n222), .b(new_n206), .c(new_n139), .d(new_n202), .o1(new_n223));
  oaoi03aa1n02x5               g128(.a(\a[18] ), .b(\b[17] ), .c(new_n215), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  nanp02aa1n02x5               g130(.a(\b[18] ), .b(\a[19] ), .o1(new_n226));
  nor042aa1n06x5               g131(.a(\b[18] ), .b(\a[19] ), .o1(new_n227));
  norb02aa1n09x5               g132(.a(new_n226), .b(new_n227), .out0(new_n228));
  xnbna2aa1n03x5               g133(.a(new_n228), .b(new_n223), .c(new_n225), .out0(\s[19] ));
  xnrc02aa1n02x5               g134(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g135(.a(new_n227), .o1(new_n231));
  aoai13aa1n03x5               g136(.a(new_n228), .b(new_n224), .c(new_n208), .d(new_n222), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[20] ), .b(\b[19] ), .out0(new_n233));
  inv000aa1n02x5               g138(.a(new_n228), .o1(new_n234));
  tech160nm_finand02aa1n05x5   g139(.a(\b[19] ), .b(\a[20] ), .o1(new_n235));
  oai022aa1d18x5               g140(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n236));
  norb02aa1n03x5               g141(.a(new_n235), .b(new_n236), .out0(new_n237));
  aoai13aa1n02x5               g142(.a(new_n237), .b(new_n234), .c(new_n223), .d(new_n225), .o1(new_n238));
  aoai13aa1n03x5               g143(.a(new_n238), .b(new_n233), .c(new_n232), .d(new_n231), .o1(\s[20] ));
  nano32aa1n03x7               g144(.a(new_n234), .b(new_n233), .c(new_n209), .d(new_n217), .out0(new_n240));
  aoai13aa1n02x7               g145(.a(new_n240), .b(new_n206), .c(new_n139), .d(new_n202), .o1(new_n241));
  aoi022aa1n06x5               g146(.a(\b[18] ), .b(\a[19] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n242));
  oai112aa1n06x5               g147(.a(new_n237), .b(new_n242), .c(new_n219), .d(new_n218), .o1(new_n243));
  nand42aa1n02x5               g148(.a(new_n236), .b(new_n235), .o1(new_n244));
  nand02aa1d06x5               g149(.a(new_n243), .b(new_n244), .o1(new_n245));
  nand02aa1d28x5               g150(.a(\b[20] ), .b(\a[21] ), .o1(new_n246));
  nor002aa1d32x5               g151(.a(\b[20] ), .b(\a[21] ), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n246), .b(new_n247), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n245), .c(new_n208), .d(new_n240), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n248), .o1(new_n250));
  and003aa1n02x5               g155(.a(new_n243), .b(new_n250), .c(new_n244), .o(new_n251));
  aobi12aa1n02x7               g156(.a(new_n249), .b(new_n251), .c(new_n241), .out0(\s[21] ));
  inv000aa1d42x5               g157(.a(new_n247), .o1(new_n253));
  nor002aa1d32x5               g158(.a(\b[21] ), .b(\a[22] ), .o1(new_n254));
  nand42aa1d28x5               g159(.a(\b[21] ), .b(\a[22] ), .o1(new_n255));
  norb02aa1n02x5               g160(.a(new_n255), .b(new_n254), .out0(new_n256));
  inv000aa1d42x5               g161(.a(new_n245), .o1(new_n257));
  nona22aa1d36x5               g162(.a(new_n255), .b(new_n254), .c(new_n247), .out0(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  aoai13aa1n02x7               g164(.a(new_n259), .b(new_n250), .c(new_n241), .d(new_n257), .o1(new_n260));
  aoai13aa1n03x5               g165(.a(new_n260), .b(new_n256), .c(new_n249), .d(new_n253), .o1(\s[22] ));
  norb02aa1n02x5               g166(.a(new_n233), .b(new_n234), .out0(new_n262));
  nano23aa1d15x5               g167(.a(new_n254), .b(new_n247), .c(new_n255), .d(new_n246), .out0(new_n263));
  and003aa1n02x5               g168(.a(new_n222), .b(new_n262), .c(new_n263), .o(new_n264));
  aoai13aa1n02x7               g169(.a(new_n264), .b(new_n206), .c(new_n139), .d(new_n202), .o1(new_n265));
  aoi022aa1n12x5               g170(.a(new_n245), .b(new_n263), .c(new_n255), .d(new_n258), .o1(new_n266));
  inv000aa1n02x5               g171(.a(new_n266), .o1(new_n267));
  nor042aa1d18x5               g172(.a(\b[22] ), .b(\a[23] ), .o1(new_n268));
  nand02aa1n08x5               g173(.a(\b[22] ), .b(\a[23] ), .o1(new_n269));
  norb02aa1d21x5               g174(.a(new_n269), .b(new_n268), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n267), .c(new_n208), .d(new_n264), .o1(new_n271));
  aoi122aa1n02x5               g176(.a(new_n270), .b(new_n255), .c(new_n258), .d(new_n245), .e(new_n263), .o1(new_n272));
  aobi12aa1n02x7               g177(.a(new_n271), .b(new_n272), .c(new_n265), .out0(\s[23] ));
  inv000aa1d42x5               g178(.a(new_n268), .o1(new_n274));
  xorc02aa1n12x5               g179(.a(\a[24] ), .b(\b[23] ), .out0(new_n275));
  inv000aa1d42x5               g180(.a(new_n270), .o1(new_n276));
  nand22aa1n04x5               g181(.a(\b[23] ), .b(\a[24] ), .o1(new_n277));
  inv020aa1n04x5               g182(.a(new_n277), .o1(new_n278));
  oaih22aa1n06x5               g183(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n279));
  norp02aa1n02x5               g184(.a(new_n279), .b(new_n278), .o1(new_n280));
  aoai13aa1n02x7               g185(.a(new_n280), .b(new_n276), .c(new_n265), .d(new_n266), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n275), .c(new_n271), .d(new_n274), .o1(\s[24] ));
  nand23aa1n06x5               g187(.a(new_n263), .b(new_n270), .c(new_n275), .o1(new_n283));
  nano22aa1n03x7               g188(.a(new_n283), .b(new_n222), .c(new_n262), .out0(new_n284));
  aoai13aa1n02x7               g189(.a(new_n284), .b(new_n206), .c(new_n139), .d(new_n202), .o1(new_n285));
  tech160nm_fioai012aa1n04x5   g190(.a(new_n255), .b(\b[22] ), .c(\a[23] ), .o1(new_n286));
  oai012aa1n03x5               g191(.a(new_n269), .b(\b[23] ), .c(\a[24] ), .o1(new_n287));
  norp03aa1d12x5               g192(.a(new_n287), .b(new_n286), .c(new_n278), .o1(new_n288));
  aoi022aa1d24x5               g193(.a(new_n288), .b(new_n258), .c(new_n277), .d(new_n279), .o1(new_n289));
  aoai13aa1n12x5               g194(.a(new_n289), .b(new_n283), .c(new_n243), .d(new_n244), .o1(new_n290));
  xorc02aa1n12x5               g195(.a(\a[25] ), .b(\b[24] ), .out0(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n290), .c(new_n208), .d(new_n284), .o1(new_n292));
  inv040aa1n02x5               g197(.a(new_n283), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n291), .o1(new_n294));
  oaib12aa1n02x5               g199(.a(new_n294), .b(new_n278), .c(new_n279), .out0(new_n295));
  aoi122aa1n02x5               g200(.a(new_n295), .b(new_n288), .c(new_n258), .d(new_n293), .e(new_n245), .o1(new_n296));
  aobi12aa1n03x7               g201(.a(new_n292), .b(new_n296), .c(new_n285), .out0(\s[25] ));
  orn002aa1n02x5               g202(.a(\a[25] ), .b(\b[24] ), .o(new_n298));
  xorc02aa1n02x5               g203(.a(\a[26] ), .b(\b[25] ), .out0(new_n299));
  inv000aa1d42x5               g204(.a(new_n290), .o1(new_n300));
  and002aa1n02x5               g205(.a(\b[25] ), .b(\a[26] ), .o(new_n301));
  oai022aa1n02x5               g206(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n302));
  norp02aa1n02x5               g207(.a(new_n302), .b(new_n301), .o1(new_n303));
  aoai13aa1n02x5               g208(.a(new_n303), .b(new_n294), .c(new_n285), .d(new_n300), .o1(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n299), .c(new_n292), .d(new_n298), .o1(\s[26] ));
  and002aa1n06x5               g210(.a(new_n291), .b(new_n299), .o(new_n306));
  nanp03aa1n03x5               g211(.a(new_n240), .b(new_n293), .c(new_n306), .o1(new_n307));
  inv000aa1n02x5               g212(.a(new_n307), .o1(new_n308));
  aoai13aa1n04x5               g213(.a(new_n308), .b(new_n206), .c(new_n139), .d(new_n202), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n301), .o1(new_n310));
  aoi022aa1n12x5               g215(.a(new_n290), .b(new_n306), .c(new_n310), .d(new_n302), .o1(new_n311));
  aoai13aa1n06x5               g216(.a(new_n311), .b(new_n307), .c(new_n203), .d(new_n207), .o1(new_n312));
  xorc02aa1n12x5               g217(.a(\a[27] ), .b(\b[26] ), .out0(new_n313));
  aoi122aa1n02x7               g218(.a(new_n313), .b(new_n310), .c(new_n302), .d(new_n290), .e(new_n306), .o1(new_n314));
  aoi022aa1n03x5               g219(.a(new_n312), .b(new_n313), .c(new_n309), .d(new_n314), .o1(\s[27] ));
  norp02aa1n02x5               g220(.a(\b[26] ), .b(\a[27] ), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n316), .o1(new_n317));
  nand42aa1n02x5               g222(.a(new_n312), .b(new_n313), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[28] ), .b(\b[27] ), .out0(new_n319));
  inv000aa1d42x5               g224(.a(new_n313), .o1(new_n320));
  nand02aa1n03x5               g225(.a(\b[27] ), .b(\a[28] ), .o1(new_n321));
  oai022aa1d24x5               g226(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n322));
  norb02aa1n06x4               g227(.a(new_n321), .b(new_n322), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n320), .c(new_n309), .d(new_n311), .o1(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n319), .c(new_n318), .d(new_n317), .o1(\s[28] ));
  nanp02aa1n02x5               g230(.a(\b[28] ), .b(\a[29] ), .o1(new_n326));
  nor042aa1n02x5               g231(.a(\b[28] ), .b(\a[29] ), .o1(new_n327));
  nanb02aa1n06x5               g232(.a(new_n327), .b(new_n326), .out0(new_n328));
  and002aa1n02x5               g233(.a(new_n319), .b(new_n313), .o(new_n329));
  oaoi03aa1n02x5               g234(.a(\a[28] ), .b(\b[27] ), .c(new_n317), .o1(new_n330));
  aoai13aa1n02x7               g235(.a(new_n328), .b(new_n330), .c(new_n312), .d(new_n329), .o1(new_n331));
  inv000aa1d42x5               g236(.a(new_n329), .o1(new_n332));
  aoi012aa1n02x5               g237(.a(new_n328), .b(new_n321), .c(new_n322), .o1(new_n333));
  aoai13aa1n02x5               g238(.a(new_n333), .b(new_n332), .c(new_n309), .d(new_n311), .o1(new_n334));
  nanp02aa1n03x5               g239(.a(new_n331), .b(new_n334), .o1(\s[29] ));
  xorb03aa1n02x5               g240(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g241(.a(new_n328), .b(new_n313), .c(new_n319), .out0(new_n337));
  nand42aa1n02x5               g242(.a(new_n312), .b(new_n337), .o1(new_n338));
  aoi013aa1n02x4               g243(.a(new_n327), .b(new_n322), .c(new_n326), .d(new_n321), .o1(new_n339));
  norp02aa1n02x5               g244(.a(\b[29] ), .b(\a[30] ), .o1(new_n340));
  nanp02aa1n02x5               g245(.a(\b[29] ), .b(\a[30] ), .o1(new_n341));
  norb02aa1n02x5               g246(.a(new_n341), .b(new_n340), .out0(new_n342));
  inv000aa1n02x5               g247(.a(new_n337), .o1(new_n343));
  nona23aa1n03x5               g248(.a(new_n321), .b(new_n326), .c(new_n323), .d(new_n327), .out0(new_n344));
  nona23aa1n09x5               g249(.a(new_n344), .b(new_n341), .c(new_n340), .d(new_n327), .out0(new_n345));
  inv000aa1d42x5               g250(.a(new_n345), .o1(new_n346));
  aoai13aa1n03x5               g251(.a(new_n346), .b(new_n343), .c(new_n309), .d(new_n311), .o1(new_n347));
  aoai13aa1n03x5               g252(.a(new_n347), .b(new_n342), .c(new_n338), .d(new_n339), .o1(\s[30] ));
  nano32aa1d12x5               g253(.a(new_n328), .b(new_n319), .c(new_n313), .d(new_n342), .out0(new_n349));
  and002aa1n02x5               g254(.a(new_n345), .b(new_n341), .o(new_n350));
  xnrc02aa1n02x5               g255(.a(\b[30] ), .b(\a[31] ), .out0(new_n351));
  aoai13aa1n03x5               g256(.a(new_n351), .b(new_n350), .c(new_n312), .d(new_n349), .o1(new_n352));
  inv000aa1d42x5               g257(.a(new_n349), .o1(new_n353));
  aoi012aa1n02x5               g258(.a(new_n351), .b(new_n345), .c(new_n341), .o1(new_n354));
  aoai13aa1n03x5               g259(.a(new_n354), .b(new_n353), .c(new_n309), .d(new_n311), .o1(new_n355));
  nanp02aa1n03x5               g260(.a(new_n352), .b(new_n355), .o1(\s[31] ));
  xnbna2aa1n03x5               g261(.a(new_n133), .b(new_n101), .c(new_n98), .out0(\s[3] ));
  inv000aa1d42x5               g262(.a(new_n104), .o1(new_n358));
  aoai13aa1n02x5               g263(.a(new_n133), .b(new_n97), .c(new_n100), .d(new_n99), .o1(new_n359));
  xnbna2aa1n03x5               g264(.a(new_n132), .b(new_n359), .c(new_n358), .out0(\s[4] ));
  xobna2aa1n03x5               g265(.a(new_n135), .b(new_n134), .c(new_n107), .out0(\s[5] ));
  aoi012aa1n02x5               g266(.a(new_n109), .b(new_n108), .c(new_n110), .o1(new_n362));
  aoai13aa1n02x5               g267(.a(new_n146), .b(new_n135), .c(new_n134), .d(new_n107), .o1(new_n363));
  oai012aa1n02x5               g268(.a(new_n363), .b(new_n362), .c(new_n136), .o1(\s[6] ));
  oai012aa1n02x5               g269(.a(new_n111), .b(new_n112), .c(new_n109), .o1(new_n365));
  nanb02aa1n02x5               g270(.a(new_n113), .b(new_n108), .out0(new_n366));
  xnbna2aa1n03x5               g271(.a(new_n117), .b(new_n366), .c(new_n365), .out0(\s[7] ));
  orn002aa1n02x5               g272(.a(\a[7] ), .b(\b[6] ), .o(new_n368));
  aob012aa1n02x5               g273(.a(new_n117), .b(new_n366), .c(new_n365), .out0(new_n369));
  xnbna2aa1n03x5               g274(.a(new_n114), .b(new_n369), .c(new_n368), .out0(\s[8] ));
  aoi113aa1n02x5               g275(.a(new_n129), .b(new_n125), .c(new_n122), .d(new_n120), .e(new_n114), .o1(new_n371));
  aoi022aa1n02x5               g276(.a(new_n139), .b(new_n129), .c(new_n119), .d(new_n371), .o1(\s[9] ));
endmodule


