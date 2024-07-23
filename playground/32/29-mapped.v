// Benchmark "adder" written by ABC on Thu Jul 18 04:36:43 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n169, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n208, new_n209,
    new_n210, new_n211, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n226, new_n227, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n254, new_n256,
    new_n257, new_n258, new_n259, new_n260, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n348, new_n349, new_n350, new_n351, new_n352,
    new_n354, new_n356, new_n357, new_n359, new_n360, new_n361, new_n363,
    new_n364, new_n365;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nanp02aa1n04x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  norb02aa1n06x5               g005(.a(new_n100), .b(new_n99), .out0(new_n101));
  nor042aa1n04x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aoi022aa1n02x7               g007(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n103));
  oai022aa1n09x5               g008(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n104));
  oaoi13aa1n09x5               g009(.a(new_n104), .b(new_n101), .c(new_n103), .d(new_n102), .o1(new_n105));
  tech160nm_finand02aa1n05x5   g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  oai012aa1n03x5               g011(.a(new_n106), .b(\b[5] ), .c(\a[6] ), .o1(new_n107));
  nanp02aa1n04x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nor002aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nand02aa1n06x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nano22aa1n03x7               g015(.a(new_n109), .b(new_n108), .c(new_n110), .out0(new_n111));
  nand42aa1n08x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  oaih12aa1n02x5               g017(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .o1(new_n113));
  nor042aa1n06x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n04x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  norb02aa1n03x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nona23aa1n09x5               g021(.a(new_n111), .b(new_n116), .c(new_n113), .d(new_n107), .out0(new_n117));
  nanb03aa1n02x5               g022(.a(new_n114), .b(new_n115), .c(new_n110), .out0(new_n118));
  inv000aa1d42x5               g023(.a(\b[7] ), .o1(new_n119));
  nanb02aa1n02x5               g024(.a(\a[8] ), .b(new_n119), .out0(new_n120));
  oai022aa1n02x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  nand43aa1n02x5               g026(.a(new_n121), .b(new_n120), .c(new_n108), .o1(new_n122));
  aob012aa1n02x5               g027(.a(new_n120), .b(new_n114), .c(new_n110), .out0(new_n123));
  oab012aa1n09x5               g028(.a(new_n123), .b(new_n122), .c(new_n118), .out0(new_n124));
  oai012aa1n12x5               g029(.a(new_n124), .b(new_n117), .c(new_n105), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(new_n97), .b(new_n98), .c(new_n125), .o1(new_n126));
  xnrb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  inv000aa1d42x5               g032(.a(\b[2] ), .o1(new_n128));
  nanb02aa1n03x5               g033(.a(\a[3] ), .b(new_n128), .out0(new_n129));
  nand42aa1n02x5               g034(.a(new_n129), .b(new_n100), .o1(new_n130));
  inv030aa1n02x5               g035(.a(new_n102), .o1(new_n131));
  nand42aa1n06x5               g036(.a(\b[0] ), .b(\a[1] ), .o1(new_n132));
  aob012aa1n06x5               g037(.a(new_n132), .b(\b[1] ), .c(\a[2] ), .out0(new_n133));
  inv040aa1n02x5               g038(.a(new_n104), .o1(new_n134));
  aoai13aa1n06x5               g039(.a(new_n134), .b(new_n130), .c(new_n133), .d(new_n131), .o1(new_n135));
  nano23aa1n02x5               g040(.a(new_n107), .b(new_n109), .c(new_n110), .d(new_n108), .out0(new_n136));
  norb03aa1n02x5               g041(.a(new_n115), .b(new_n113), .c(new_n114), .out0(new_n137));
  nand23aa1n03x5               g042(.a(new_n135), .b(new_n136), .c(new_n137), .o1(new_n138));
  nor002aa1d32x5               g043(.a(\b[9] ), .b(\a[10] ), .o1(new_n139));
  nanp02aa1n12x5               g044(.a(\b[9] ), .b(\a[10] ), .o1(new_n140));
  aoai13aa1n06x5               g045(.a(new_n140), .b(new_n139), .c(new_n97), .d(new_n98), .o1(new_n141));
  nor002aa1d32x5               g046(.a(\b[8] ), .b(\a[9] ), .o1(new_n142));
  nand42aa1n03x5               g047(.a(\b[8] ), .b(\a[9] ), .o1(new_n143));
  nona23aa1n09x5               g048(.a(new_n140), .b(new_n143), .c(new_n142), .d(new_n139), .out0(new_n144));
  aoai13aa1n03x5               g049(.a(new_n141), .b(new_n144), .c(new_n138), .d(new_n124), .o1(new_n145));
  xorb03aa1n02x5               g050(.a(new_n145), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  nand02aa1d28x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  inv040aa1d32x5               g054(.a(\a[11] ), .o1(new_n150));
  inv000aa1d42x5               g055(.a(\b[10] ), .o1(new_n151));
  oaoi03aa1n03x5               g056(.a(new_n150), .b(new_n151), .c(new_n145), .o1(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n152), .b(new_n148), .c(new_n149), .out0(\s[12] ));
  nor022aa1n16x5               g058(.a(\b[10] ), .b(\a[11] ), .o1(new_n154));
  nand02aa1d16x5               g059(.a(\b[10] ), .b(\a[11] ), .o1(new_n155));
  nona23aa1n09x5               g060(.a(new_n149), .b(new_n155), .c(new_n154), .d(new_n147), .out0(new_n156));
  nor042aa1n03x5               g061(.a(new_n156), .b(new_n144), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(new_n125), .b(new_n157), .o1(new_n158));
  nanb03aa1n12x5               g063(.a(new_n147), .b(new_n149), .c(new_n155), .out0(new_n159));
  nanp02aa1n09x5               g064(.a(new_n151), .b(new_n150), .o1(new_n160));
  oai112aa1n06x5               g065(.a(new_n140), .b(new_n160), .c(new_n139), .d(new_n142), .o1(new_n161));
  aoi012aa1d18x5               g066(.a(new_n147), .b(new_n154), .c(new_n149), .o1(new_n162));
  oai012aa1d24x5               g067(.a(new_n162), .b(new_n161), .c(new_n159), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  nor002aa1n10x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nanp02aa1n06x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n158), .c(new_n164), .out0(\s[13] ));
  aoai13aa1n02x5               g073(.a(new_n167), .b(new_n163), .c(new_n125), .d(new_n157), .o1(new_n169));
  oaih12aa1n02x5               g074(.a(new_n169), .b(\b[12] ), .c(\a[13] ), .o1(new_n170));
  xorb03aa1n02x5               g075(.a(new_n170), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n12x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nanp02aa1n04x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nano23aa1n03x7               g078(.a(new_n165), .b(new_n172), .c(new_n173), .d(new_n166), .out0(new_n174));
  aoai13aa1n06x5               g079(.a(new_n174), .b(new_n163), .c(new_n125), .d(new_n157), .o1(new_n175));
  oa0012aa1n02x5               g080(.a(new_n173), .b(new_n172), .c(new_n165), .o(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  nor042aa1n06x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nand02aa1n04x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norb02aa1n03x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  xnbna2aa1n03x5               g085(.a(new_n180), .b(new_n175), .c(new_n177), .out0(\s[15] ));
  nand02aa1n04x5               g086(.a(new_n175), .b(new_n177), .o1(new_n182));
  nor042aa1n06x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nand22aa1n12x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  norb02aa1n06x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  inv000aa1d42x5               g090(.a(new_n185), .o1(new_n186));
  aoai13aa1n02x5               g091(.a(new_n186), .b(new_n178), .c(new_n182), .d(new_n179), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(new_n182), .b(new_n180), .o1(new_n188));
  nona22aa1n03x5               g093(.a(new_n188), .b(new_n186), .c(new_n178), .out0(new_n189));
  nanp02aa1n02x5               g094(.a(new_n189), .b(new_n187), .o1(\s[16] ));
  nano23aa1n02x4               g095(.a(new_n142), .b(new_n139), .c(new_n140), .d(new_n143), .out0(new_n191));
  nona23aa1n09x5               g096(.a(new_n184), .b(new_n179), .c(new_n178), .d(new_n183), .out0(new_n192));
  nano23aa1n06x5               g097(.a(new_n192), .b(new_n156), .c(new_n191), .d(new_n174), .out0(new_n193));
  nanp02aa1n02x5               g098(.a(new_n125), .b(new_n193), .o1(new_n194));
  xorc02aa1n02x5               g099(.a(\a[17] ), .b(\b[16] ), .out0(new_n195));
  nona23aa1n03x5               g100(.a(new_n173), .b(new_n166), .c(new_n165), .d(new_n172), .out0(new_n196));
  nor042aa1n02x5               g101(.a(new_n192), .b(new_n196), .o1(new_n197));
  nanb03aa1n02x5               g102(.a(new_n183), .b(new_n184), .c(new_n179), .out0(new_n198));
  orn002aa1n03x5               g103(.a(\a[15] ), .b(\b[14] ), .o(new_n199));
  oai112aa1n02x5               g104(.a(new_n199), .b(new_n173), .c(new_n172), .d(new_n165), .o1(new_n200));
  oaoi03aa1n02x5               g105(.a(\a[16] ), .b(\b[15] ), .c(new_n199), .o1(new_n201));
  oabi12aa1n02x5               g106(.a(new_n201), .b(new_n198), .c(new_n200), .out0(new_n202));
  aoi112aa1n02x5               g107(.a(new_n202), .b(new_n195), .c(new_n163), .d(new_n197), .o1(new_n203));
  nand42aa1n02x5               g108(.a(new_n157), .b(new_n197), .o1(new_n204));
  tech160nm_fiaoi012aa1n05x5   g109(.a(new_n202), .b(new_n163), .c(new_n197), .o1(new_n205));
  aoai13aa1n09x5               g110(.a(new_n205), .b(new_n204), .c(new_n138), .d(new_n124), .o1(new_n206));
  aoi022aa1n02x5               g111(.a(new_n206), .b(new_n195), .c(new_n194), .d(new_n203), .o1(\s[17] ));
  inv000aa1d42x5               g112(.a(\a[17] ), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(\b[16] ), .b(new_n208), .out0(new_n209));
  oaib12aa1n06x5               g114(.a(new_n206), .b(new_n208), .c(\b[16] ), .out0(new_n210));
  xorc02aa1n02x5               g115(.a(\a[18] ), .b(\b[17] ), .out0(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n210), .c(new_n209), .out0(\s[18] ));
  inv000aa1n02x5               g117(.a(new_n141), .o1(new_n213));
  inv000aa1n02x5               g118(.a(new_n149), .o1(new_n214));
  oai012aa1n02x5               g119(.a(new_n155), .b(\b[11] ), .c(\a[12] ), .o1(new_n215));
  nona32aa1n02x4               g120(.a(new_n213), .b(new_n215), .c(new_n214), .d(new_n154), .out0(new_n216));
  nanp03aa1n02x5               g121(.a(new_n174), .b(new_n180), .c(new_n185), .o1(new_n217));
  oab012aa1n02x5               g122(.a(new_n201), .b(new_n200), .c(new_n198), .out0(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n217), .c(new_n216), .d(new_n162), .o1(new_n219));
  inv020aa1n04x5               g124(.a(\a[18] ), .o1(new_n220));
  xroi22aa1d06x4               g125(.a(new_n208), .b(\b[16] ), .c(new_n220), .d(\b[17] ), .out0(new_n221));
  aoai13aa1n06x5               g126(.a(new_n221), .b(new_n219), .c(new_n125), .d(new_n193), .o1(new_n222));
  tech160nm_fioaoi03aa1n03p5x5 g127(.a(\a[18] ), .b(\b[17] ), .c(new_n209), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  nor042aa1d18x5               g129(.a(\b[18] ), .b(\a[19] ), .o1(new_n225));
  nand02aa1n06x5               g130(.a(\b[18] ), .b(\a[19] ), .o1(new_n226));
  norb02aa1n03x5               g131(.a(new_n226), .b(new_n225), .out0(new_n227));
  xnbna2aa1n03x5               g132(.a(new_n227), .b(new_n222), .c(new_n224), .out0(\s[19] ));
  xnrc02aa1n02x5               g133(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand02aa1n03x5               g134(.a(new_n222), .b(new_n224), .o1(new_n230));
  nor042aa1n09x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  nand02aa1d16x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  nanb02aa1n02x5               g137(.a(new_n231), .b(new_n232), .out0(new_n233));
  aoai13aa1n03x5               g138(.a(new_n233), .b(new_n225), .c(new_n230), .d(new_n226), .o1(new_n234));
  aoai13aa1n03x5               g139(.a(new_n227), .b(new_n223), .c(new_n206), .d(new_n221), .o1(new_n235));
  nona22aa1n02x4               g140(.a(new_n235), .b(new_n233), .c(new_n225), .out0(new_n236));
  nanp02aa1n03x5               g141(.a(new_n234), .b(new_n236), .o1(\s[20] ));
  nanb03aa1n06x5               g142(.a(new_n233), .b(new_n221), .c(new_n227), .out0(new_n238));
  nanb02aa1n02x5               g143(.a(new_n238), .b(new_n206), .out0(new_n239));
  aoi012aa1n12x5               g144(.a(new_n219), .b(new_n125), .c(new_n193), .o1(new_n240));
  nanb03aa1n02x5               g145(.a(new_n231), .b(new_n232), .c(new_n226), .out0(new_n241));
  nand02aa1n04x5               g146(.a(\b[17] ), .b(\a[18] ), .o1(new_n242));
  oaih22aa1d12x5               g147(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n243));
  oai112aa1n03x5               g148(.a(new_n243), .b(new_n242), .c(\b[18] ), .d(\a[19] ), .o1(new_n244));
  tech160nm_fiaoi012aa1n03p5x5 g149(.a(new_n231), .b(new_n225), .c(new_n232), .o1(new_n245));
  oaih12aa1n06x5               g150(.a(new_n245), .b(new_n244), .c(new_n241), .o1(new_n246));
  oabi12aa1n06x5               g151(.a(new_n246), .b(new_n240), .c(new_n238), .out0(new_n247));
  xnrc02aa1n12x5               g152(.a(\b[20] ), .b(\a[21] ), .out0(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  nano22aa1n02x4               g154(.a(new_n231), .b(new_n226), .c(new_n232), .out0(new_n250));
  oai012aa1n02x5               g155(.a(new_n242), .b(\b[18] ), .c(\a[19] ), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n243), .b(new_n251), .out0(new_n252));
  inv000aa1n03x5               g157(.a(new_n245), .o1(new_n253));
  aoi112aa1n02x5               g158(.a(new_n253), .b(new_n249), .c(new_n252), .d(new_n250), .o1(new_n254));
  aoi022aa1n02x5               g159(.a(new_n247), .b(new_n249), .c(new_n239), .d(new_n254), .o1(\s[21] ));
  nor042aa1n04x5               g160(.a(\b[20] ), .b(\a[21] ), .o1(new_n256));
  xnrc02aa1n12x5               g161(.a(\b[21] ), .b(\a[22] ), .out0(new_n257));
  aoai13aa1n03x5               g162(.a(new_n257), .b(new_n256), .c(new_n247), .d(new_n249), .o1(new_n258));
  nand42aa1n02x5               g163(.a(new_n247), .b(new_n249), .o1(new_n259));
  nona22aa1n03x5               g164(.a(new_n259), .b(new_n257), .c(new_n256), .out0(new_n260));
  nanp02aa1n03x5               g165(.a(new_n260), .b(new_n258), .o1(\s[22] ));
  nor042aa1n09x5               g166(.a(new_n257), .b(new_n248), .o1(new_n262));
  nano32aa1n03x7               g167(.a(new_n233), .b(new_n221), .c(new_n262), .d(new_n227), .out0(new_n263));
  inv000aa1d42x5               g168(.a(\a[22] ), .o1(new_n264));
  inv000aa1d42x5               g169(.a(\b[21] ), .o1(new_n265));
  oao003aa1n02x5               g170(.a(new_n264), .b(new_n265), .c(new_n256), .carry(new_n266));
  aoi012aa1d24x5               g171(.a(new_n266), .b(new_n246), .c(new_n262), .o1(new_n267));
  oaib12aa1n06x5               g172(.a(new_n267), .b(new_n240), .c(new_n263), .out0(new_n268));
  xorb03aa1n02x5               g173(.a(new_n268), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g174(.a(\b[22] ), .b(\a[23] ), .o1(new_n270));
  xorc02aa1n12x5               g175(.a(\a[23] ), .b(\b[22] ), .out0(new_n271));
  xnrc02aa1n12x5               g176(.a(\b[23] ), .b(\a[24] ), .out0(new_n272));
  aoai13aa1n03x5               g177(.a(new_n272), .b(new_n270), .c(new_n268), .d(new_n271), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n267), .o1(new_n274));
  aoai13aa1n02x5               g179(.a(new_n271), .b(new_n274), .c(new_n206), .d(new_n263), .o1(new_n275));
  nona22aa1n02x4               g180(.a(new_n275), .b(new_n272), .c(new_n270), .out0(new_n276));
  nanp02aa1n03x5               g181(.a(new_n273), .b(new_n276), .o1(\s[24] ));
  aoai13aa1n06x5               g182(.a(new_n262), .b(new_n253), .c(new_n252), .d(new_n250), .o1(new_n278));
  inv000aa1n02x5               g183(.a(new_n266), .o1(new_n279));
  norb02aa1n03x5               g184(.a(new_n271), .b(new_n272), .out0(new_n280));
  inv000aa1n02x5               g185(.a(new_n280), .o1(new_n281));
  aoi112aa1n02x5               g186(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n282));
  oab012aa1n02x4               g187(.a(new_n282), .b(\a[24] ), .c(\b[23] ), .out0(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n281), .c(new_n278), .d(new_n279), .o1(new_n284));
  inv040aa1n03x5               g189(.a(new_n284), .o1(new_n285));
  inv000aa1n02x5               g190(.a(new_n262), .o1(new_n286));
  nona32aa1n03x5               g191(.a(new_n206), .b(new_n281), .c(new_n286), .d(new_n238), .out0(new_n287));
  nanp02aa1n03x5               g192(.a(new_n287), .b(new_n285), .o1(new_n288));
  xorc02aa1n12x5               g193(.a(\a[25] ), .b(\b[24] ), .out0(new_n289));
  aoai13aa1n06x5               g194(.a(new_n280), .b(new_n266), .c(new_n246), .d(new_n262), .o1(new_n290));
  nano22aa1n03x7               g195(.a(new_n240), .b(new_n263), .c(new_n280), .out0(new_n291));
  nano23aa1n02x4               g196(.a(new_n291), .b(new_n289), .c(new_n283), .d(new_n290), .out0(new_n292));
  tech160nm_fiaoi012aa1n02p5x5 g197(.a(new_n292), .b(new_n288), .c(new_n289), .o1(\s[25] ));
  norp02aa1n02x5               g198(.a(\b[24] ), .b(\a[25] ), .o1(new_n294));
  xnrc02aa1n12x5               g199(.a(\b[25] ), .b(\a[26] ), .out0(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n294), .c(new_n288), .d(new_n289), .o1(new_n296));
  oaih12aa1n02x5               g201(.a(new_n289), .b(new_n291), .c(new_n284), .o1(new_n297));
  nona22aa1n03x5               g202(.a(new_n297), .b(new_n295), .c(new_n294), .out0(new_n298));
  nanp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[26] ));
  nor042aa1n06x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  and002aa1n02x5               g205(.a(\b[26] ), .b(\a[27] ), .o(new_n301));
  nor002aa1n03x5               g206(.a(new_n301), .b(new_n300), .o1(new_n302));
  nona23aa1n02x4               g207(.a(new_n289), .b(new_n271), .c(new_n295), .d(new_n272), .out0(new_n303));
  nona32aa1n09x5               g208(.a(new_n206), .b(new_n303), .c(new_n286), .d(new_n238), .out0(new_n304));
  nanp02aa1n02x5               g209(.a(\b[25] ), .b(\a[26] ), .o1(new_n305));
  norb02aa1n06x5               g210(.a(new_n289), .b(new_n295), .out0(new_n306));
  oai022aa1n02x5               g211(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n307));
  aoi022aa1n09x5               g212(.a(new_n284), .b(new_n306), .c(new_n305), .d(new_n307), .o1(new_n308));
  xnbna2aa1n03x5               g213(.a(new_n302), .b(new_n308), .c(new_n304), .out0(\s[27] ));
  nano32aa1n03x7               g214(.a(new_n240), .b(new_n306), .c(new_n263), .d(new_n280), .out0(new_n310));
  inv000aa1n02x5               g215(.a(new_n306), .o1(new_n311));
  nanp02aa1n02x5               g216(.a(new_n307), .b(new_n305), .o1(new_n312));
  aoai13aa1n06x5               g217(.a(new_n312), .b(new_n311), .c(new_n290), .d(new_n283), .o1(new_n313));
  oabi12aa1n03x5               g218(.a(new_n301), .b(new_n310), .c(new_n313), .out0(new_n314));
  inv000aa1d42x5               g219(.a(new_n300), .o1(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n301), .c(new_n308), .d(new_n304), .o1(new_n316));
  xorc02aa1n02x5               g221(.a(\a[28] ), .b(\b[27] ), .out0(new_n317));
  norp02aa1n02x5               g222(.a(new_n317), .b(new_n300), .o1(new_n318));
  aoi022aa1n03x5               g223(.a(new_n316), .b(new_n317), .c(new_n314), .d(new_n318), .o1(\s[28] ));
  and002aa1n02x5               g224(.a(new_n317), .b(new_n302), .o(new_n320));
  oai012aa1n03x5               g225(.a(new_n320), .b(new_n310), .c(new_n313), .o1(new_n321));
  inv040aa1n03x5               g226(.a(new_n320), .o1(new_n322));
  oao003aa1n02x5               g227(.a(\a[28] ), .b(\b[27] ), .c(new_n315), .carry(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n322), .c(new_n308), .d(new_n304), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .out0(new_n325));
  norb02aa1n02x5               g230(.a(new_n323), .b(new_n325), .out0(new_n326));
  aoi022aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n321), .d(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g232(.a(new_n132), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g233(.a(new_n317), .b(new_n325), .c(new_n302), .o(new_n329));
  oai012aa1n03x5               g234(.a(new_n329), .b(new_n310), .c(new_n313), .o1(new_n330));
  inv000aa1n02x5               g235(.a(new_n329), .o1(new_n331));
  oao003aa1n02x5               g236(.a(\a[29] ), .b(\b[28] ), .c(new_n323), .carry(new_n332));
  aoai13aa1n03x5               g237(.a(new_n332), .b(new_n331), .c(new_n308), .d(new_n304), .o1(new_n333));
  xorc02aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .out0(new_n334));
  norb02aa1n02x5               g239(.a(new_n332), .b(new_n334), .out0(new_n335));
  aoi022aa1n03x5               g240(.a(new_n333), .b(new_n334), .c(new_n330), .d(new_n335), .o1(\s[30] ));
  nano22aa1n02x4               g241(.a(new_n322), .b(new_n325), .c(new_n334), .out0(new_n337));
  tech160nm_fioai012aa1n04x5   g242(.a(new_n337), .b(new_n310), .c(new_n313), .o1(new_n338));
  xorc02aa1n02x5               g243(.a(\a[31] ), .b(\b[30] ), .out0(new_n339));
  and002aa1n02x5               g244(.a(\b[29] ), .b(\a[30] ), .o(new_n340));
  oabi12aa1n02x5               g245(.a(new_n339), .b(\a[30] ), .c(\b[29] ), .out0(new_n341));
  oab012aa1n02x4               g246(.a(new_n341), .b(new_n332), .c(new_n340), .out0(new_n342));
  inv000aa1n02x5               g247(.a(new_n337), .o1(new_n343));
  oao003aa1n02x5               g248(.a(\a[30] ), .b(\b[29] ), .c(new_n332), .carry(new_n344));
  aoai13aa1n03x5               g249(.a(new_n344), .b(new_n343), .c(new_n308), .d(new_n304), .o1(new_n345));
  aoi022aa1n03x5               g250(.a(new_n345), .b(new_n339), .c(new_n338), .d(new_n342), .o1(\s[31] ));
  xnbna2aa1n03x5               g251(.a(new_n101), .b(new_n133), .c(new_n131), .out0(\s[3] ));
  inv000aa1d42x5               g252(.a(\a[4] ), .o1(new_n348));
  inv000aa1d42x5               g253(.a(\b[3] ), .o1(new_n349));
  nanp02aa1n02x5               g254(.a(new_n349), .b(new_n348), .o1(new_n350));
  aoi012aa1n02x5               g255(.a(new_n130), .b(new_n133), .c(new_n131), .o1(new_n351));
  aoi112aa1n02x5               g256(.a(new_n351), .b(new_n99), .c(new_n350), .d(new_n112), .o1(new_n352));
  aoi013aa1n02x4               g257(.a(new_n352), .b(new_n135), .c(new_n112), .d(new_n350), .o1(\s[4] ));
  xorc02aa1n02x5               g258(.a(\a[5] ), .b(\b[4] ), .out0(new_n354));
  xobna2aa1n03x5               g259(.a(new_n354), .b(new_n135), .c(new_n112), .out0(\s[5] ));
  aoai13aa1n06x5               g260(.a(new_n354), .b(new_n105), .c(\a[4] ), .d(\b[3] ), .o1(new_n356));
  xorc02aa1n02x5               g261(.a(\a[6] ), .b(\b[5] ), .out0(new_n357));
  xobna2aa1n03x5               g262(.a(new_n357), .b(new_n356), .c(new_n106), .out0(\s[6] ));
  aobi12aa1n06x5               g263(.a(new_n357), .b(new_n356), .c(new_n106), .out0(new_n359));
  aoi012aa1n02x5               g264(.a(new_n359), .b(\a[6] ), .c(\b[5] ), .o1(new_n360));
  nano23aa1n02x4               g265(.a(new_n359), .b(new_n114), .c(new_n108), .d(new_n115), .out0(new_n361));
  oab012aa1n02x4               g266(.a(new_n361), .b(new_n360), .c(new_n116), .out0(\s[7] ));
  nanp02aa1n02x5               g267(.a(new_n120), .b(new_n110), .o1(new_n363));
  oai012aa1n02x5               g268(.a(new_n363), .b(new_n361), .c(new_n114), .o1(new_n364));
  orn003aa1n03x5               g269(.a(new_n361), .b(new_n363), .c(new_n114), .o(new_n365));
  nanp02aa1n02x5               g270(.a(new_n365), .b(new_n364), .o1(\s[8] ));
  xorb03aa1n02x5               g271(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


