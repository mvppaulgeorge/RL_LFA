// Benchmark "adder" written by ABC on Wed Jul 17 15:20:46 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n333, new_n334, new_n335, new_n337, new_n338,
    new_n341, new_n342, new_n343, new_n344, new_n346, new_n347, new_n349,
    new_n350;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  and002aa1n12x5               g001(.a(\b[0] ), .b(\a[1] ), .o(new_n97));
  tech160nm_fioaoi03aa1n03p5x5 g002(.a(\a[2] ), .b(\b[1] ), .c(new_n97), .o1(new_n98));
  tech160nm_finand02aa1n05x5   g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nor042aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  norb02aa1n03x5               g005(.a(new_n99), .b(new_n100), .out0(new_n101));
  xorc02aa1n06x5               g006(.a(\a[3] ), .b(\b[2] ), .out0(new_n102));
  nanp03aa1n06x5               g007(.a(new_n98), .b(new_n102), .c(new_n101), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  aoi012aa1n06x5               g009(.a(new_n100), .b(new_n104), .c(new_n99), .o1(new_n105));
  nand42aa1n08x5               g010(.a(\b[7] ), .b(\a[8] ), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[7] ), .b(\a[8] ), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  nand02aa1n06x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nano23aa1n03x5               g014(.a(new_n108), .b(new_n107), .c(new_n109), .d(new_n106), .out0(new_n110));
  tech160nm_fixorc02aa1n03p5x5 g015(.a(\a[5] ), .b(\b[4] ), .out0(new_n111));
  xorc02aa1n02x5               g016(.a(\a[6] ), .b(\b[5] ), .out0(new_n112));
  nand23aa1n03x5               g017(.a(new_n110), .b(new_n111), .c(new_n112), .o1(new_n113));
  oai022aa1n02x7               g018(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n114));
  inv000aa1n02x5               g019(.a(new_n109), .o1(new_n115));
  nor022aa1n02x5               g020(.a(new_n114), .b(new_n115), .o1(new_n116));
  nand02aa1d08x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  oai022aa1n02x7               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  and003aa1n03x5               g023(.a(new_n118), .b(new_n117), .c(new_n106), .o(new_n119));
  aoi022aa1n06x5               g024(.a(new_n119), .b(new_n116), .c(new_n106), .d(new_n114), .o1(new_n120));
  aoai13aa1n12x5               g025(.a(new_n120), .b(new_n113), .c(new_n103), .d(new_n105), .o1(new_n121));
  nor002aa1n03x5               g026(.a(\b[8] ), .b(\a[9] ), .o1(new_n122));
  tech160nm_fixorc02aa1n03p5x5 g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  xorc02aa1n06x5               g028(.a(\a[10] ), .b(\b[9] ), .out0(new_n124));
  aoi112aa1n02x5               g029(.a(new_n122), .b(new_n124), .c(new_n121), .d(new_n123), .o1(new_n125));
  aoai13aa1n06x5               g030(.a(new_n124), .b(new_n122), .c(new_n121), .d(new_n123), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(\s[10] ));
  nor002aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  oaih12aa1n02x5               g034(.a(new_n129), .b(new_n128), .c(new_n122), .o1(new_n130));
  nor042aa1n06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand42aa1n03x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n12x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n126), .c(new_n130), .out0(\s[11] ));
  aob012aa1n02x5               g039(.a(new_n133), .b(new_n126), .c(new_n130), .out0(new_n135));
  nor042aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  tech160nm_finand02aa1n03p5x5 g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n06x4               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  aoib12aa1n02x5               g043(.a(new_n131), .b(new_n137), .c(new_n136), .out0(new_n139));
  inv000aa1d42x5               g044(.a(new_n131), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n133), .o1(new_n141));
  aoai13aa1n02x5               g046(.a(new_n140), .b(new_n141), .c(new_n126), .d(new_n130), .o1(new_n142));
  aoi022aa1n02x5               g047(.a(new_n142), .b(new_n138), .c(new_n135), .d(new_n139), .o1(\s[12] ));
  nano23aa1n06x5               g048(.a(new_n131), .b(new_n136), .c(new_n137), .d(new_n132), .out0(new_n144));
  nand23aa1d12x5               g049(.a(new_n144), .b(new_n123), .c(new_n124), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  tech160nm_fiaoi012aa1n05x5   g051(.a(new_n136), .b(new_n131), .c(new_n137), .o1(new_n147));
  nanb03aa1n09x5               g052(.a(new_n130), .b(new_n138), .c(new_n133), .out0(new_n148));
  nanp02aa1n02x5               g053(.a(new_n148), .b(new_n147), .o1(new_n149));
  xnrc02aa1n12x5               g054(.a(\b[12] ), .b(\a[13] ), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  aoai13aa1n06x5               g056(.a(new_n151), .b(new_n149), .c(new_n121), .d(new_n146), .o1(new_n152));
  aoi112aa1n02x5               g057(.a(new_n151), .b(new_n149), .c(new_n121), .d(new_n146), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(\s[13] ));
  norp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  xnrc02aa1n12x5               g061(.a(\b[13] ), .b(\a[14] ), .out0(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n152), .c(new_n156), .out0(\s[14] ));
  nor042aa1n06x5               g064(.a(new_n157), .b(new_n150), .o1(new_n160));
  aoai13aa1n06x5               g065(.a(new_n160), .b(new_n149), .c(new_n121), .d(new_n146), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  oai022aa1n04x7               g067(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(new_n163), .b(new_n162), .o1(new_n164));
  xorc02aa1n02x5               g069(.a(\a[15] ), .b(\b[14] ), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n161), .c(new_n164), .out0(\s[15] ));
  aob012aa1n02x5               g071(.a(new_n165), .b(new_n161), .c(new_n164), .out0(new_n167));
  xorc02aa1n02x5               g072(.a(\a[16] ), .b(\b[15] ), .out0(new_n168));
  inv000aa1d42x5               g073(.a(\a[15] ), .o1(new_n169));
  inv000aa1d42x5               g074(.a(\b[14] ), .o1(new_n170));
  inv000aa1d42x5               g075(.a(\a[16] ), .o1(new_n171));
  inv000aa1d42x5               g076(.a(\b[15] ), .o1(new_n172));
  nanp02aa1n03x5               g077(.a(new_n172), .b(new_n171), .o1(new_n173));
  nand42aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  aoi022aa1n02x5               g079(.a(new_n173), .b(new_n174), .c(new_n170), .d(new_n169), .o1(new_n175));
  norp02aa1n09x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  xnrc02aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n177), .b(new_n178), .c(new_n161), .d(new_n164), .o1(new_n179));
  aoi022aa1n02x5               g084(.a(new_n179), .b(new_n168), .c(new_n167), .d(new_n175), .o1(\s[16] ));
  nano22aa1n02x5               g085(.a(new_n178), .b(new_n173), .c(new_n174), .out0(new_n181));
  nano22aa1n12x5               g086(.a(new_n145), .b(new_n160), .c(new_n181), .out0(new_n182));
  nanp02aa1n09x5               g087(.a(new_n121), .b(new_n182), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n160), .b(new_n181), .o1(new_n184));
  oai112aa1n02x5               g089(.a(new_n173), .b(new_n174), .c(new_n170), .d(new_n169), .o1(new_n185));
  oai112aa1n02x5               g090(.a(new_n163), .b(new_n162), .c(\b[14] ), .d(\a[15] ), .o1(new_n186));
  oaoi03aa1n02x5               g091(.a(new_n171), .b(new_n172), .c(new_n176), .o1(new_n187));
  oa0012aa1n09x5               g092(.a(new_n187), .b(new_n186), .c(new_n185), .o(new_n188));
  aoai13aa1n12x5               g093(.a(new_n188), .b(new_n184), .c(new_n148), .d(new_n147), .o1(new_n189));
  inv030aa1n06x5               g094(.a(new_n189), .o1(new_n190));
  nand02aa1d10x5               g095(.a(new_n183), .b(new_n190), .o1(new_n191));
  xorc02aa1n02x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  inv000aa1d42x5               g097(.a(new_n188), .o1(new_n193));
  aoi113aa1n02x5               g098(.a(new_n193), .b(new_n192), .c(new_n149), .d(new_n160), .e(new_n181), .o1(new_n194));
  aoi022aa1n02x5               g099(.a(new_n191), .b(new_n192), .c(new_n183), .d(new_n194), .o1(\s[17] ));
  inv000aa1d42x5               g100(.a(\a[17] ), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(\b[16] ), .b(new_n196), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n192), .b(new_n189), .c(new_n121), .d(new_n182), .o1(new_n198));
  nor042aa1n03x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nand42aa1n04x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  norb02aa1n06x4               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  xnbna2aa1n03x5               g106(.a(new_n201), .b(new_n198), .c(new_n197), .out0(\s[18] ));
  and002aa1n02x5               g107(.a(new_n192), .b(new_n201), .o(new_n203));
  aoai13aa1n03x5               g108(.a(new_n203), .b(new_n189), .c(new_n121), .d(new_n182), .o1(new_n204));
  oaoi03aa1n02x5               g109(.a(\a[18] ), .b(\b[17] ), .c(new_n197), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  nor042aa1d18x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nanp02aa1n04x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  norb02aa1d21x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n204), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n06x5               g116(.a(new_n209), .b(new_n205), .c(new_n191), .d(new_n203), .o1(new_n212));
  nor042aa1n04x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nand22aa1n09x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  inv000aa1d42x5               g120(.a(\a[19] ), .o1(new_n216));
  inv000aa1d42x5               g121(.a(\b[18] ), .o1(new_n217));
  aboi22aa1n03x5               g122(.a(new_n213), .b(new_n214), .c(new_n216), .d(new_n217), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n207), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n209), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n219), .b(new_n220), .c(new_n204), .d(new_n206), .o1(new_n221));
  aoi022aa1n03x5               g126(.a(new_n221), .b(new_n215), .c(new_n212), .d(new_n218), .o1(\s[20] ));
  nano32aa1n03x7               g127(.a(new_n220), .b(new_n192), .c(new_n215), .d(new_n201), .out0(new_n223));
  aoai13aa1n03x5               g128(.a(new_n223), .b(new_n189), .c(new_n121), .d(new_n182), .o1(new_n224));
  nanb03aa1n12x5               g129(.a(new_n213), .b(new_n214), .c(new_n208), .out0(new_n225));
  nor042aa1n02x5               g130(.a(\b[16] ), .b(\a[17] ), .o1(new_n226));
  oai112aa1n06x5               g131(.a(new_n219), .b(new_n200), .c(new_n199), .d(new_n226), .o1(new_n227));
  aoi012aa1d18x5               g132(.a(new_n213), .b(new_n207), .c(new_n214), .o1(new_n228));
  oai012aa1d24x5               g133(.a(new_n228), .b(new_n227), .c(new_n225), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  nor042aa1n06x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  nanp02aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  norb02aa1n06x4               g137(.a(new_n232), .b(new_n231), .out0(new_n233));
  xnbna2aa1n03x5               g138(.a(new_n233), .b(new_n224), .c(new_n230), .out0(\s[21] ));
  aoai13aa1n06x5               g139(.a(new_n233), .b(new_n229), .c(new_n191), .d(new_n223), .o1(new_n235));
  nor042aa1n02x5               g140(.a(\b[21] ), .b(\a[22] ), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(\b[21] ), .b(\a[22] ), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(new_n238));
  aoib12aa1n02x5               g143(.a(new_n231), .b(new_n237), .c(new_n236), .out0(new_n239));
  inv000aa1n03x5               g144(.a(new_n231), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n233), .o1(new_n241));
  aoai13aa1n02x5               g146(.a(new_n240), .b(new_n241), .c(new_n224), .d(new_n230), .o1(new_n242));
  aoi022aa1n03x5               g147(.a(new_n242), .b(new_n238), .c(new_n235), .d(new_n239), .o1(\s[22] ));
  inv000aa1n02x5               g148(.a(new_n223), .o1(new_n244));
  nano22aa1n03x7               g149(.a(new_n244), .b(new_n233), .c(new_n238), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n189), .c(new_n121), .d(new_n182), .o1(new_n246));
  nano23aa1n06x5               g151(.a(new_n231), .b(new_n236), .c(new_n237), .d(new_n232), .out0(new_n247));
  oaoi03aa1n12x5               g152(.a(\a[22] ), .b(\b[21] ), .c(new_n240), .o1(new_n248));
  tech160nm_fiaoi012aa1n05x5   g153(.a(new_n248), .b(new_n229), .c(new_n247), .o1(new_n249));
  xorc02aa1n12x5               g154(.a(\a[23] ), .b(\b[22] ), .out0(new_n250));
  xnbna2aa1n03x5               g155(.a(new_n250), .b(new_n246), .c(new_n249), .out0(\s[23] ));
  inv000aa1d42x5               g156(.a(new_n249), .o1(new_n252));
  aoai13aa1n06x5               g157(.a(new_n250), .b(new_n252), .c(new_n191), .d(new_n245), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .out0(new_n254));
  nor042aa1n06x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  norp02aa1n02x5               g160(.a(new_n254), .b(new_n255), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n255), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n250), .o1(new_n258));
  aoai13aa1n03x5               g163(.a(new_n257), .b(new_n258), .c(new_n246), .d(new_n249), .o1(new_n259));
  aoi022aa1n02x5               g164(.a(new_n259), .b(new_n254), .c(new_n253), .d(new_n256), .o1(\s[24] ));
  and002aa1n18x5               g165(.a(new_n254), .b(new_n250), .o(new_n261));
  nano22aa1n03x7               g166(.a(new_n244), .b(new_n261), .c(new_n247), .out0(new_n262));
  aoai13aa1n04x5               g167(.a(new_n262), .b(new_n189), .c(new_n121), .d(new_n182), .o1(new_n263));
  aoai13aa1n06x5               g168(.a(new_n261), .b(new_n248), .c(new_n229), .d(new_n247), .o1(new_n264));
  nano22aa1n03x5               g169(.a(new_n213), .b(new_n208), .c(new_n214), .out0(new_n265));
  oai012aa1n02x5               g170(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .o1(new_n266));
  oab012aa1n02x4               g171(.a(new_n266), .b(new_n226), .c(new_n199), .out0(new_n267));
  inv020aa1n02x5               g172(.a(new_n228), .o1(new_n268));
  aoai13aa1n06x5               g173(.a(new_n247), .b(new_n268), .c(new_n267), .d(new_n265), .o1(new_n269));
  inv030aa1n02x5               g174(.a(new_n248), .o1(new_n270));
  inv000aa1n06x5               g175(.a(new_n261), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[24] ), .b(\b[23] ), .c(new_n257), .carry(new_n272));
  aoai13aa1n12x5               g177(.a(new_n272), .b(new_n271), .c(new_n269), .d(new_n270), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  xorc02aa1n12x5               g179(.a(\a[25] ), .b(\b[24] ), .out0(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  aoi012aa1n02x5               g181(.a(new_n276), .b(new_n263), .c(new_n274), .o1(new_n277));
  norb02aa1n02x5               g182(.a(new_n272), .b(new_n275), .out0(new_n278));
  aoi013aa1n02x4               g183(.a(new_n277), .b(new_n264), .c(new_n263), .d(new_n278), .o1(\s[25] ));
  aoai13aa1n06x5               g184(.a(new_n275), .b(new_n273), .c(new_n191), .d(new_n262), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[26] ), .b(\b[25] ), .out0(new_n281));
  nor042aa1n03x5               g186(.a(\b[24] ), .b(\a[25] ), .o1(new_n282));
  norp02aa1n02x5               g187(.a(new_n281), .b(new_n282), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n282), .o1(new_n284));
  aoai13aa1n04x5               g189(.a(new_n284), .b(new_n276), .c(new_n263), .d(new_n274), .o1(new_n285));
  aoi022aa1n02x5               g190(.a(new_n285), .b(new_n281), .c(new_n280), .d(new_n283), .o1(\s[26] ));
  and002aa1n12x5               g191(.a(new_n281), .b(new_n275), .o(new_n287));
  nano32aa1n03x7               g192(.a(new_n244), .b(new_n287), .c(new_n247), .d(new_n261), .out0(new_n288));
  aoai13aa1n06x5               g193(.a(new_n288), .b(new_n189), .c(new_n121), .d(new_n182), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n287), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[26] ), .b(\b[25] ), .c(new_n284), .carry(new_n291));
  aoai13aa1n04x5               g196(.a(new_n291), .b(new_n290), .c(new_n264), .d(new_n272), .o1(new_n292));
  xorc02aa1n12x5               g197(.a(\a[27] ), .b(\b[26] ), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n292), .c(new_n191), .d(new_n288), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n291), .o1(new_n295));
  aoi112aa1n02x5               g200(.a(new_n293), .b(new_n295), .c(new_n273), .d(new_n287), .o1(new_n296));
  aobi12aa1n02x7               g201(.a(new_n294), .b(new_n296), .c(new_n289), .out0(\s[27] ));
  tech160nm_fixorc02aa1n02p5x5 g202(.a(\a[28] ), .b(\b[27] ), .out0(new_n298));
  norp02aa1n02x5               g203(.a(\b[26] ), .b(\a[27] ), .o1(new_n299));
  norp02aa1n02x5               g204(.a(new_n298), .b(new_n299), .o1(new_n300));
  tech160nm_fiaoi012aa1n05x5   g205(.a(new_n295), .b(new_n273), .c(new_n287), .o1(new_n301));
  inv000aa1n03x5               g206(.a(new_n299), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n293), .o1(new_n303));
  aoai13aa1n03x5               g208(.a(new_n302), .b(new_n303), .c(new_n301), .d(new_n289), .o1(new_n304));
  aoi022aa1n03x5               g209(.a(new_n304), .b(new_n298), .c(new_n294), .d(new_n300), .o1(\s[28] ));
  and002aa1n02x5               g210(.a(new_n298), .b(new_n293), .o(new_n306));
  aoai13aa1n02x7               g211(.a(new_n306), .b(new_n292), .c(new_n191), .d(new_n288), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n306), .o1(new_n308));
  oao003aa1n02x5               g213(.a(\a[28] ), .b(\b[27] ), .c(new_n302), .carry(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n308), .c(new_n301), .d(new_n289), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[29] ), .b(\b[28] ), .out0(new_n311));
  norb02aa1n02x5               g216(.a(new_n309), .b(new_n311), .out0(new_n312));
  aoi022aa1n03x5               g217(.a(new_n310), .b(new_n311), .c(new_n307), .d(new_n312), .o1(\s[29] ));
  xnrb03aa1n02x5               g218(.a(new_n97), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g219(.a(new_n303), .b(new_n298), .c(new_n311), .out0(new_n315));
  aoai13aa1n02x7               g220(.a(new_n315), .b(new_n292), .c(new_n191), .d(new_n288), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n315), .o1(new_n317));
  oao003aa1n02x5               g222(.a(\a[29] ), .b(\b[28] ), .c(new_n309), .carry(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n317), .c(new_n301), .d(new_n289), .o1(new_n319));
  xorc02aa1n02x5               g224(.a(\a[30] ), .b(\b[29] ), .out0(new_n320));
  norb02aa1n02x5               g225(.a(new_n318), .b(new_n320), .out0(new_n321));
  aoi022aa1n03x5               g226(.a(new_n319), .b(new_n320), .c(new_n316), .d(new_n321), .o1(\s[30] ));
  nano32aa1n03x7               g227(.a(new_n303), .b(new_n320), .c(new_n298), .d(new_n311), .out0(new_n323));
  aoai13aa1n02x7               g228(.a(new_n323), .b(new_n292), .c(new_n191), .d(new_n288), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[31] ), .b(\b[30] ), .out0(new_n325));
  and002aa1n02x5               g230(.a(\b[29] ), .b(\a[30] ), .o(new_n326));
  oabi12aa1n02x5               g231(.a(new_n325), .b(\a[30] ), .c(\b[29] ), .out0(new_n327));
  oab012aa1n02x4               g232(.a(new_n327), .b(new_n318), .c(new_n326), .out0(new_n328));
  inv000aa1d42x5               g233(.a(new_n323), .o1(new_n329));
  oao003aa1n02x5               g234(.a(\a[30] ), .b(\b[29] ), .c(new_n318), .carry(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n329), .c(new_n301), .d(new_n289), .o1(new_n331));
  aoi022aa1n03x5               g236(.a(new_n331), .b(new_n325), .c(new_n324), .d(new_n328), .o1(\s[31] ));
  orn002aa1n02x5               g237(.a(\a[2] ), .b(\b[1] ), .o(new_n333));
  nanp02aa1n02x5               g238(.a(\b[1] ), .b(\a[2] ), .o1(new_n334));
  nanb03aa1n02x5               g239(.a(new_n97), .b(new_n333), .c(new_n334), .out0(new_n335));
  xnbna2aa1n03x5               g240(.a(new_n102), .b(new_n335), .c(new_n333), .out0(\s[3] ));
  nanp02aa1n02x5               g241(.a(new_n103), .b(new_n105), .o1(new_n337));
  aoi112aa1n02x5               g242(.a(new_n104), .b(new_n101), .c(new_n98), .d(new_n102), .o1(new_n338));
  oaoi13aa1n02x5               g243(.a(new_n338), .b(new_n337), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xnbna2aa1n03x5               g244(.a(new_n111), .b(new_n103), .c(new_n105), .out0(\s[5] ));
  nanp02aa1n02x5               g245(.a(new_n337), .b(new_n111), .o1(new_n341));
  oaoi13aa1n02x5               g246(.a(new_n112), .b(new_n341), .c(\a[5] ), .d(\b[4] ), .o1(new_n342));
  inv000aa1d42x5               g247(.a(new_n117), .o1(new_n343));
  nona22aa1n02x4               g248(.a(new_n341), .b(new_n118), .c(new_n343), .out0(new_n344));
  nanb02aa1n02x5               g249(.a(new_n342), .b(new_n344), .out0(\s[6] ));
  aboi22aa1n03x5               g250(.a(new_n108), .b(new_n109), .c(new_n344), .d(new_n117), .out0(new_n346));
  nona32aa1n02x4               g251(.a(new_n344), .b(new_n343), .c(new_n115), .d(new_n108), .out0(new_n347));
  norb02aa1n02x5               g252(.a(new_n347), .b(new_n346), .out0(\s[7] ));
  norb02aa1n02x5               g253(.a(new_n106), .b(new_n107), .out0(new_n349));
  orn002aa1n02x5               g254(.a(\a[7] ), .b(\b[6] ), .o(new_n350));
  xnbna2aa1n03x5               g255(.a(new_n349), .b(new_n347), .c(new_n350), .out0(\s[8] ));
  xorb03aa1n02x5               g256(.a(new_n121), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


