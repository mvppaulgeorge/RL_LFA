// Benchmark "adder" written by ABC on Wed Jul 17 21:37:57 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n322, new_n324;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1d18x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nor002aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  tech160nm_fioai012aa1n04x5   g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  nor002aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n06x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  aoi012aa1n02x5               g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oaih12aa1n06x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  xnrc02aa1n02x5               g014(.a(\b[5] ), .b(\a[6] ), .out0(new_n110));
  nor022aa1n06x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand22aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  norp02aa1n06x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n06x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor043aa1n03x5               g021(.a(new_n115), .b(new_n116), .c(new_n110), .o1(new_n117));
  nanp02aa1n06x5               g022(.a(new_n109), .b(new_n117), .o1(new_n118));
  oai022aa1n02x5               g023(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n119));
  inv040aa1d32x5               g024(.a(\a[5] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[4] ), .o1(new_n121));
  nand42aa1n03x5               g026(.a(new_n121), .b(new_n120), .o1(new_n122));
  oaoi03aa1n03x5               g027(.a(\a[6] ), .b(\b[5] ), .c(new_n122), .o1(new_n123));
  aboi22aa1n12x5               g028(.a(new_n115), .b(new_n123), .c(new_n112), .d(new_n119), .out0(new_n124));
  nand02aa1d06x5               g029(.a(new_n118), .b(new_n124), .o1(new_n125));
  nand42aa1n03x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  tech160nm_fiaoi012aa1n05x5   g031(.a(new_n98), .b(new_n125), .c(new_n126), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  nand02aa1d16x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  inv000aa1n02x5               g034(.a(new_n129), .o1(new_n130));
  nor042aa1d18x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand42aa1n16x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  inv000aa1d42x5               g037(.a(new_n132), .o1(new_n133));
  oaib12aa1n06x5               g038(.a(new_n127), .b(\b[9] ), .c(new_n97), .out0(new_n134));
  nona32aa1n06x5               g039(.a(new_n134), .b(new_n133), .c(new_n131), .d(new_n130), .out0(new_n135));
  inv040aa1n06x5               g040(.a(new_n131), .o1(new_n136));
  aoi022aa1n02x5               g041(.a(new_n134), .b(new_n129), .c(new_n136), .d(new_n132), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n135), .b(new_n137), .out0(\s[11] ));
  nor042aa1n06x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand42aa1n06x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  aobi12aa1n03x5               g046(.a(new_n141), .b(new_n135), .c(new_n136), .out0(new_n142));
  aoi113aa1n02x5               g047(.a(new_n131), .b(new_n141), .c(new_n134), .d(new_n132), .e(new_n129), .o1(new_n143));
  norp02aa1n03x5               g048(.a(new_n142), .b(new_n143), .o1(\s[12] ));
  nor002aa1n16x5               g049(.a(\b[9] ), .b(\a[10] ), .o1(new_n145));
  norb03aa1n03x5               g050(.a(new_n126), .b(new_n98), .c(new_n131), .out0(new_n146));
  nano22aa1d15x5               g051(.a(new_n139), .b(new_n132), .c(new_n140), .out0(new_n147));
  nona23aa1n12x5               g052(.a(new_n147), .b(new_n146), .c(new_n145), .d(new_n130), .out0(new_n148));
  nanb03aa1n12x5               g053(.a(new_n139), .b(new_n140), .c(new_n132), .out0(new_n149));
  oai112aa1n06x5               g054(.a(new_n136), .b(new_n129), .c(new_n145), .d(new_n98), .o1(new_n150));
  oaoi03aa1n09x5               g055(.a(\a[12] ), .b(\b[11] ), .c(new_n136), .o1(new_n151));
  oabi12aa1n18x5               g056(.a(new_n151), .b(new_n149), .c(new_n150), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n148), .c(new_n118), .d(new_n124), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  orn002aa1n24x5               g060(.a(\a[13] ), .b(\b[12] ), .o(new_n156));
  nand42aa1n03x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  aobi12aa1n02x5               g062(.a(new_n156), .b(new_n154), .c(new_n157), .out0(new_n158));
  xnrb03aa1n03x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n12x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nand42aa1n06x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  tech160nm_fixnrc02aa1n04x5   g067(.a(\b[13] ), .b(\a[14] ), .out0(new_n163));
  nano22aa1n12x5               g068(.a(new_n163), .b(new_n156), .c(new_n157), .out0(new_n164));
  oaoi03aa1n09x5               g069(.a(\a[14] ), .b(\b[13] ), .c(new_n156), .o1(new_n165));
  aoai13aa1n03x5               g070(.a(new_n162), .b(new_n165), .c(new_n154), .d(new_n164), .o1(new_n166));
  aoi112aa1n02x5               g071(.a(new_n162), .b(new_n165), .c(new_n154), .d(new_n164), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(\s[15] ));
  inv000aa1d42x5               g073(.a(new_n160), .o1(new_n169));
  nor042aa1n04x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand42aa1n08x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n166), .c(new_n169), .out0(\s[16] ));
  inv000aa1d42x5               g078(.a(\a[7] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(\b[6] ), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n112), .b(new_n111), .c(new_n174), .d(new_n175), .o1(new_n176));
  oaib12aa1n03x5               g081(.a(new_n176), .b(new_n115), .c(new_n123), .out0(new_n177));
  nano23aa1d15x5               g082(.a(new_n160), .b(new_n170), .c(new_n171), .d(new_n161), .out0(new_n178));
  nano22aa1d15x5               g083(.a(new_n148), .b(new_n164), .c(new_n178), .out0(new_n179));
  aoai13aa1n06x5               g084(.a(new_n179), .b(new_n177), .c(new_n109), .d(new_n117), .o1(new_n180));
  aoai13aa1n12x5               g085(.a(new_n178), .b(new_n165), .c(new_n152), .d(new_n164), .o1(new_n181));
  aoi012aa1n09x5               g086(.a(new_n170), .b(new_n160), .c(new_n171), .o1(new_n182));
  nanp03aa1d12x5               g087(.a(new_n180), .b(new_n181), .c(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g089(.a(\a[18] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\a[17] ), .o1(new_n186));
  inv000aa1d42x5               g091(.a(\b[16] ), .o1(new_n187));
  oaoi03aa1n03x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  inv000aa1n02x5               g094(.a(new_n165), .o1(new_n190));
  inv000aa1d42x5               g095(.a(new_n178), .o1(new_n191));
  oai012aa1n02x5               g096(.a(new_n129), .b(\b[10] ), .c(\a[11] ), .o1(new_n192));
  oab012aa1n02x4               g097(.a(new_n192), .b(new_n98), .c(new_n145), .out0(new_n193));
  aoai13aa1n04x5               g098(.a(new_n164), .b(new_n151), .c(new_n193), .d(new_n147), .o1(new_n194));
  aoai13aa1n12x5               g099(.a(new_n182), .b(new_n191), .c(new_n194), .d(new_n190), .o1(new_n195));
  xroi22aa1d06x4               g100(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n195), .c(new_n125), .d(new_n179), .o1(new_n197));
  oai022aa1d24x5               g102(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n198));
  oaib12aa1n18x5               g103(.a(new_n198), .b(new_n185), .c(\b[17] ), .out0(new_n199));
  nor002aa1d32x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nand42aa1n10x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n197), .c(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g110(.a(new_n200), .o1(new_n206));
  inv040aa1n08x5               g111(.a(new_n199), .o1(new_n207));
  aoai13aa1n06x5               g112(.a(new_n203), .b(new_n207), .c(new_n183), .d(new_n196), .o1(new_n208));
  norp02aa1n24x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand42aa1d28x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanb02aa1n02x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  aoi012aa1n03x5               g116(.a(new_n211), .b(new_n208), .c(new_n206), .o1(new_n212));
  aoi012aa1n03x5               g117(.a(new_n202), .b(new_n197), .c(new_n199), .o1(new_n213));
  nano22aa1n03x5               g118(.a(new_n213), .b(new_n206), .c(new_n211), .out0(new_n214));
  norp02aa1n03x5               g119(.a(new_n212), .b(new_n214), .o1(\s[20] ));
  nano23aa1n09x5               g120(.a(new_n200), .b(new_n209), .c(new_n210), .d(new_n201), .out0(new_n216));
  nand02aa1d04x5               g121(.a(new_n196), .b(new_n216), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n195), .c(new_n125), .d(new_n179), .o1(new_n219));
  nona23aa1d18x5               g124(.a(new_n210), .b(new_n201), .c(new_n200), .d(new_n209), .out0(new_n220));
  aoi012aa1n12x5               g125(.a(new_n209), .b(new_n200), .c(new_n210), .o1(new_n221));
  oai012aa1d24x5               g126(.a(new_n221), .b(new_n220), .c(new_n199), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  xnbna2aa1n03x5               g129(.a(new_n224), .b(new_n219), .c(new_n223), .out0(\s[21] ));
  orn002aa1n24x5               g130(.a(\a[21] ), .b(\b[20] ), .o(new_n226));
  aoai13aa1n03x5               g131(.a(new_n224), .b(new_n222), .c(new_n183), .d(new_n218), .o1(new_n227));
  xnrc02aa1n12x5               g132(.a(\b[21] ), .b(\a[22] ), .out0(new_n228));
  aoi012aa1n03x5               g133(.a(new_n228), .b(new_n227), .c(new_n226), .o1(new_n229));
  aobi12aa1n03x5               g134(.a(new_n224), .b(new_n219), .c(new_n223), .out0(new_n230));
  nano22aa1n03x5               g135(.a(new_n230), .b(new_n226), .c(new_n228), .out0(new_n231));
  nor002aa1n02x5               g136(.a(new_n229), .b(new_n231), .o1(\s[22] ));
  nanp02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  nano22aa1n12x5               g138(.a(new_n228), .b(new_n226), .c(new_n233), .out0(new_n234));
  and003aa1n02x5               g139(.a(new_n196), .b(new_n234), .c(new_n216), .o(new_n235));
  aoai13aa1n06x5               g140(.a(new_n235), .b(new_n195), .c(new_n125), .d(new_n179), .o1(new_n236));
  oaoi03aa1n09x5               g141(.a(\a[22] ), .b(\b[21] ), .c(new_n226), .o1(new_n237));
  aoi012aa1d18x5               g142(.a(new_n237), .b(new_n222), .c(new_n234), .o1(new_n238));
  xnrc02aa1n12x5               g143(.a(\b[22] ), .b(\a[23] ), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  xnbna2aa1n03x5               g145(.a(new_n240), .b(new_n236), .c(new_n238), .out0(\s[23] ));
  nor042aa1n06x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n238), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n240), .b(new_n244), .c(new_n183), .d(new_n235), .o1(new_n245));
  tech160nm_fixnrc02aa1n02p5x5 g150(.a(\b[23] ), .b(\a[24] ), .out0(new_n246));
  aoi012aa1n03x5               g151(.a(new_n246), .b(new_n245), .c(new_n243), .o1(new_n247));
  aoi012aa1n02x5               g152(.a(new_n239), .b(new_n236), .c(new_n238), .o1(new_n248));
  nano22aa1n03x5               g153(.a(new_n248), .b(new_n243), .c(new_n246), .out0(new_n249));
  norp02aa1n02x5               g154(.a(new_n247), .b(new_n249), .o1(\s[24] ));
  nor002aa1n03x5               g155(.a(new_n246), .b(new_n239), .o1(new_n251));
  nano22aa1n03x7               g156(.a(new_n217), .b(new_n234), .c(new_n251), .out0(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n195), .c(new_n125), .d(new_n179), .o1(new_n253));
  inv000aa1n04x5               g158(.a(new_n221), .o1(new_n254));
  aoai13aa1n09x5               g159(.a(new_n234), .b(new_n254), .c(new_n216), .d(new_n207), .o1(new_n255));
  inv000aa1n04x5               g160(.a(new_n237), .o1(new_n256));
  inv020aa1n03x5               g161(.a(new_n251), .o1(new_n257));
  oao003aa1n02x5               g162(.a(\a[24] ), .b(\b[23] ), .c(new_n243), .carry(new_n258));
  aoai13aa1n12x5               g163(.a(new_n258), .b(new_n257), .c(new_n255), .d(new_n256), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  xnrc02aa1n12x5               g165(.a(\b[24] ), .b(\a[25] ), .out0(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  xnbna2aa1n03x5               g167(.a(new_n262), .b(new_n253), .c(new_n260), .out0(\s[25] ));
  nor042aa1n03x5               g168(.a(\b[24] ), .b(\a[25] ), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  aoai13aa1n03x5               g170(.a(new_n262), .b(new_n259), .c(new_n183), .d(new_n252), .o1(new_n266));
  tech160nm_fixnrc02aa1n04x5   g171(.a(\b[25] ), .b(\a[26] ), .out0(new_n267));
  aoi012aa1n03x5               g172(.a(new_n267), .b(new_n266), .c(new_n265), .o1(new_n268));
  aoi012aa1n03x5               g173(.a(new_n261), .b(new_n253), .c(new_n260), .o1(new_n269));
  nano22aa1n03x5               g174(.a(new_n269), .b(new_n265), .c(new_n267), .out0(new_n270));
  nor002aa1n02x5               g175(.a(new_n268), .b(new_n270), .o1(\s[26] ));
  norp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  norb02aa1n02x5               g178(.a(new_n273), .b(new_n272), .out0(new_n274));
  nor042aa1n09x5               g179(.a(new_n267), .b(new_n261), .o1(new_n275));
  nano32aa1n06x5               g180(.a(new_n217), .b(new_n275), .c(new_n234), .d(new_n251), .out0(new_n276));
  aoai13aa1n09x5               g181(.a(new_n276), .b(new_n195), .c(new_n125), .d(new_n179), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .c(new_n265), .carry(new_n278));
  aobi12aa1n12x5               g183(.a(new_n278), .b(new_n259), .c(new_n275), .out0(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n274), .b(new_n279), .c(new_n277), .out0(\s[27] ));
  inv000aa1n06x5               g185(.a(new_n272), .o1(new_n281));
  aoai13aa1n06x5               g186(.a(new_n251), .b(new_n237), .c(new_n222), .d(new_n234), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n275), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n278), .b(new_n283), .c(new_n282), .d(new_n258), .o1(new_n284));
  aoai13aa1n06x5               g189(.a(new_n273), .b(new_n284), .c(new_n276), .d(new_n183), .o1(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[27] ), .b(\a[28] ), .out0(new_n286));
  aoi012aa1n03x5               g191(.a(new_n286), .b(new_n285), .c(new_n281), .o1(new_n287));
  aoi022aa1n03x5               g192(.a(new_n279), .b(new_n277), .c(\a[27] ), .d(\b[26] ), .o1(new_n288));
  nano22aa1n03x5               g193(.a(new_n288), .b(new_n281), .c(new_n286), .out0(new_n289));
  norp02aa1n03x5               g194(.a(new_n287), .b(new_n289), .o1(\s[28] ));
  nano22aa1n02x4               g195(.a(new_n286), .b(new_n281), .c(new_n273), .out0(new_n291));
  aoai13aa1n03x5               g196(.a(new_n291), .b(new_n284), .c(new_n183), .d(new_n276), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n281), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  tech160nm_fiaoi012aa1n02p5x5 g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  aobi12aa1n03x5               g200(.a(new_n291), .b(new_n279), .c(new_n277), .out0(new_n296));
  nano22aa1n03x5               g201(.a(new_n296), .b(new_n293), .c(new_n294), .out0(new_n297));
  norp02aa1n03x5               g202(.a(new_n295), .b(new_n297), .o1(\s[29] ));
  xorb03aa1n02x5               g203(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g204(.a(new_n274), .b(new_n294), .c(new_n286), .out0(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n284), .c(new_n183), .d(new_n276), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[29] ), .b(\a[30] ), .out0(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n303), .b(new_n301), .c(new_n302), .o1(new_n304));
  aobi12aa1n03x5               g209(.a(new_n300), .b(new_n279), .c(new_n277), .out0(new_n305));
  nano22aa1n03x5               g210(.a(new_n305), .b(new_n302), .c(new_n303), .out0(new_n306));
  norp02aa1n03x5               g211(.a(new_n304), .b(new_n306), .o1(\s[30] ));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  norb03aa1n02x5               g213(.a(new_n291), .b(new_n303), .c(new_n294), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n309), .b(new_n284), .c(new_n183), .d(new_n276), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n311));
  aoi012aa1n02x5               g216(.a(new_n308), .b(new_n310), .c(new_n311), .o1(new_n312));
  aobi12aa1n03x5               g217(.a(new_n309), .b(new_n279), .c(new_n277), .out0(new_n313));
  nano22aa1n03x5               g218(.a(new_n313), .b(new_n308), .c(new_n311), .out0(new_n314));
  norp02aa1n03x5               g219(.a(new_n312), .b(new_n314), .o1(\s[31] ));
  xnrb03aa1n02x5               g220(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g221(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g223(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g224(.a(new_n120), .b(new_n121), .c(new_n109), .o1(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g226(.a(\a[6] ), .b(\b[5] ), .c(new_n320), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g228(.a(new_n174), .b(new_n175), .c(new_n322), .o1(new_n324));
  xnrb03aa1n02x5               g229(.a(new_n324), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g230(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


