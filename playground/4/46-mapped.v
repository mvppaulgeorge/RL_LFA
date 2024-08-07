// Benchmark "adder" written by ABC on Wed Jul 17 14:23:52 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n319, new_n322, new_n324, new_n326;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand42aa1n16x5               g003(.a(\b[9] ), .b(\a[10] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[8] ), .o1(new_n101));
  nor022aa1n06x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand42aa1n03x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  oai112aa1n02x5               g009(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  aoi113aa1n02x5               g011(.a(new_n102), .b(new_n106), .c(new_n105), .d(new_n104), .e(new_n103), .o1(new_n107));
  xorc02aa1n02x5               g012(.a(\a[5] ), .b(\b[4] ), .out0(new_n108));
  nor002aa1n02x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nanp02aa1n04x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n04x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nano22aa1n02x4               g016(.a(new_n109), .b(new_n110), .c(new_n111), .out0(new_n112));
  nanp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nor002aa1n03x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nand42aa1n03x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nor042aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nano23aa1n06x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  nanp03aa1n02x5               g022(.a(new_n117), .b(new_n108), .c(new_n112), .o1(new_n118));
  nanb03aa1n06x5               g023(.a(new_n109), .b(new_n111), .c(new_n110), .out0(new_n119));
  nanb02aa1n03x5               g024(.a(new_n114), .b(new_n115), .out0(new_n120));
  oab012aa1n02x4               g025(.a(new_n116), .b(\a[5] ), .c(\b[4] ), .out0(new_n121));
  norp03aa1n02x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\a[7] ), .o1(new_n123));
  inv000aa1d42x5               g028(.a(\b[6] ), .o1(new_n124));
  aoai13aa1n02x5               g029(.a(new_n115), .b(new_n114), .c(new_n123), .d(new_n124), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n122), .out0(new_n126));
  oai012aa1n02x5               g031(.a(new_n126), .b(new_n118), .c(new_n107), .o1(new_n127));
  oaoi03aa1n02x5               g032(.a(new_n100), .b(new_n101), .c(new_n127), .o1(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n98), .c(new_n99), .out0(\s[10] ));
  inv000aa1n02x5               g034(.a(new_n99), .o1(new_n130));
  nona22aa1n02x4               g035(.a(new_n128), .b(new_n130), .c(new_n97), .out0(new_n131));
  nand42aa1n06x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  aoi022aa1n02x5               g039(.a(new_n131), .b(new_n99), .c(new_n134), .d(new_n132), .o1(new_n135));
  nanb03aa1d24x5               g040(.a(new_n133), .b(new_n99), .c(new_n132), .out0(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(new_n131), .b(new_n137), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n135), .out0(\s[11] ));
  nor002aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n03x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanb02aa1n09x5               g046(.a(new_n140), .b(new_n141), .out0(new_n142));
  xobna2aa1n03x5               g047(.a(new_n142), .b(new_n138), .c(new_n134), .out0(\s[12] ));
  xorc02aa1n02x5               g048(.a(\a[9] ), .b(\b[8] ), .out0(new_n144));
  nano23aa1n02x4               g049(.a(new_n142), .b(new_n136), .c(new_n144), .d(new_n98), .out0(new_n145));
  nanp02aa1n02x5               g050(.a(new_n127), .b(new_n145), .o1(new_n146));
  aoi112aa1n06x5               g051(.a(new_n130), .b(new_n97), .c(new_n100), .d(new_n101), .o1(new_n147));
  oai012aa1n02x5               g052(.a(new_n141), .b(new_n140), .c(new_n133), .o1(new_n148));
  oai013aa1d12x5               g053(.a(new_n148), .b(new_n147), .c(new_n136), .d(new_n142), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  tech160nm_fixnrc02aa1n04x5   g055(.a(\b[12] ), .b(\a[13] ), .out0(new_n151));
  xobna2aa1n03x5               g056(.a(new_n151), .b(new_n146), .c(new_n150), .out0(\s[13] ));
  inv000aa1d42x5               g057(.a(\a[14] ), .o1(new_n153));
  inv000aa1d42x5               g058(.a(\a[13] ), .o1(new_n154));
  inv000aa1d42x5               g059(.a(\b[12] ), .o1(new_n155));
  nanb02aa1n06x5               g060(.a(new_n102), .b(new_n103), .out0(new_n156));
  norp02aa1n02x5               g061(.a(\b[1] ), .b(\a[2] ), .o1(new_n157));
  nand02aa1n02x5               g062(.a(\b[0] ), .b(\a[1] ), .o1(new_n158));
  oai012aa1n03x5               g063(.a(new_n104), .b(new_n157), .c(new_n158), .o1(new_n159));
  norp02aa1n02x5               g064(.a(new_n106), .b(new_n102), .o1(new_n160));
  tech160nm_fioai012aa1n05x5   g065(.a(new_n160), .b(new_n159), .c(new_n156), .o1(new_n161));
  xnrc02aa1n02x5               g066(.a(\b[4] ), .b(\a[5] ), .out0(new_n162));
  nor022aa1n03x5               g067(.a(new_n119), .b(new_n162), .o1(new_n163));
  oai013aa1n02x4               g068(.a(new_n125), .b(new_n119), .c(new_n120), .d(new_n121), .o1(new_n164));
  aoi013aa1n06x5               g069(.a(new_n164), .b(new_n161), .c(new_n163), .d(new_n117), .o1(new_n165));
  oaib12aa1n02x5               g070(.a(new_n150), .b(new_n165), .c(new_n145), .out0(new_n166));
  oaoi03aa1n02x5               g071(.a(new_n154), .b(new_n155), .c(new_n166), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[13] ), .c(new_n153), .out0(\s[14] ));
  and002aa1n06x5               g073(.a(\b[13] ), .b(\a[14] ), .o(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  nor022aa1n16x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norb02aa1n06x4               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  aboi22aa1n03x5               g078(.a(\b[13] ), .b(new_n153), .c(new_n154), .d(new_n155), .out0(new_n174));
  aoai13aa1n04x5               g079(.a(new_n174), .b(new_n151), .c(new_n146), .d(new_n150), .o1(new_n175));
  xobna2aa1n03x5               g080(.a(new_n173), .b(new_n175), .c(new_n170), .out0(\s[15] ));
  inv000aa1d42x5               g081(.a(new_n171), .o1(new_n177));
  nanp03aa1n02x5               g082(.a(new_n175), .b(new_n170), .c(new_n173), .o1(new_n178));
  norp02aa1n06x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  norb02aa1n06x4               g085(.a(new_n180), .b(new_n179), .out0(new_n181));
  aobi12aa1n02x5               g086(.a(new_n181), .b(new_n178), .c(new_n177), .out0(new_n182));
  aoi113aa1n02x5               g087(.a(new_n171), .b(new_n181), .c(new_n175), .d(new_n173), .e(new_n170), .o1(new_n183));
  norp02aa1n02x5               g088(.a(new_n182), .b(new_n183), .o1(\s[16] ));
  nor043aa1n03x5               g089(.a(new_n136), .b(new_n142), .c(new_n97), .o1(new_n185));
  xnrc02aa1n02x5               g090(.a(\b[13] ), .b(\a[14] ), .out0(new_n186));
  nona23aa1n03x5               g091(.a(new_n180), .b(new_n172), .c(new_n171), .d(new_n179), .out0(new_n187));
  nor043aa1n06x5               g092(.a(new_n187), .b(new_n186), .c(new_n151), .o1(new_n188));
  nand23aa1n03x5               g093(.a(new_n188), .b(new_n185), .c(new_n144), .o1(new_n189));
  aoi112aa1n02x5               g094(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n190));
  nona23aa1n02x4               g095(.a(new_n173), .b(new_n181), .c(new_n174), .d(new_n169), .out0(new_n191));
  nona22aa1n02x4               g096(.a(new_n191), .b(new_n190), .c(new_n179), .out0(new_n192));
  aoi012aa1d18x5               g097(.a(new_n192), .b(new_n149), .c(new_n188), .o1(new_n193));
  oai012aa1n18x5               g098(.a(new_n193), .b(new_n165), .c(new_n189), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g100(.a(\a[18] ), .o1(new_n196));
  inv000aa1d42x5               g101(.a(\a[17] ), .o1(new_n197));
  inv000aa1d42x5               g102(.a(\b[16] ), .o1(new_n198));
  oaoi03aa1n02x5               g103(.a(new_n197), .b(new_n198), .c(new_n194), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[17] ), .c(new_n196), .out0(\s[18] ));
  nor002aa1d32x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nand02aa1n03x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  xroi22aa1d06x4               g109(.a(new_n197), .b(\b[16] ), .c(new_n196), .d(\b[17] ), .out0(new_n205));
  nand02aa1n02x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nona22aa1n02x4               g111(.a(new_n206), .b(\b[16] ), .c(\a[17] ), .out0(new_n207));
  oaib12aa1n06x5               g112(.a(new_n207), .b(\b[17] ), .c(new_n196), .out0(new_n208));
  aoai13aa1n06x5               g113(.a(new_n204), .b(new_n208), .c(new_n194), .d(new_n205), .o1(new_n209));
  aoi112aa1n02x5               g114(.a(new_n204), .b(new_n208), .c(new_n194), .d(new_n205), .o1(new_n210));
  norb02aa1n02x5               g115(.a(new_n209), .b(new_n210), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n09x5               g117(.a(new_n201), .o1(new_n213));
  norp02aa1n06x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nand22aa1n02x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  xnbna2aa1n03x5               g121(.a(new_n216), .b(new_n209), .c(new_n213), .out0(\s[20] ));
  nona23aa1n12x5               g122(.a(new_n215), .b(new_n202), .c(new_n201), .d(new_n214), .out0(new_n218));
  norb02aa1n02x5               g123(.a(new_n205), .b(new_n218), .out0(new_n219));
  norp02aa1n02x5               g124(.a(\b[17] ), .b(\a[18] ), .o1(new_n220));
  aoi013aa1n02x4               g125(.a(new_n220), .b(new_n206), .c(new_n197), .d(new_n198), .o1(new_n221));
  oaoi03aa1n12x5               g126(.a(\a[20] ), .b(\b[19] ), .c(new_n213), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  oai012aa1n12x5               g128(.a(new_n223), .b(new_n218), .c(new_n221), .o1(new_n224));
  aoi012aa1n09x5               g129(.a(new_n224), .b(new_n194), .c(new_n219), .o1(new_n225));
  xnrb03aa1n02x5               g130(.a(new_n225), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  orn002aa1n24x5               g131(.a(\a[21] ), .b(\b[20] ), .o(new_n227));
  xnrc02aa1n12x5               g132(.a(\b[20] ), .b(\a[21] ), .out0(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[21] ), .b(\a[22] ), .out0(new_n229));
  oaoi13aa1n09x5               g134(.a(new_n229), .b(new_n227), .c(new_n225), .d(new_n228), .o1(new_n230));
  oai112aa1n04x5               g135(.a(new_n229), .b(new_n227), .c(new_n225), .d(new_n228), .o1(new_n231));
  norb02aa1n03x4               g136(.a(new_n231), .b(new_n230), .out0(\s[22] ));
  inv020aa1n02x5               g137(.a(new_n219), .o1(new_n233));
  nona32aa1n03x5               g138(.a(new_n194), .b(new_n229), .c(new_n228), .d(new_n233), .out0(new_n234));
  nor042aa1n06x5               g139(.a(new_n229), .b(new_n228), .o1(new_n235));
  oaoi03aa1n06x5               g140(.a(\a[22] ), .b(\b[21] ), .c(new_n227), .o1(new_n236));
  aoi012aa1d24x5               g141(.a(new_n236), .b(new_n224), .c(new_n235), .o1(new_n237));
  xnrc02aa1n12x5               g142(.a(\b[22] ), .b(\a[23] ), .out0(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  xnbna2aa1n03x5               g144(.a(new_n239), .b(new_n234), .c(new_n237), .out0(\s[23] ));
  orn002aa1n24x5               g145(.a(\a[23] ), .b(\b[22] ), .o(new_n241));
  oaoi13aa1n04x5               g146(.a(new_n233), .b(new_n193), .c(new_n165), .d(new_n189), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n237), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n239), .b(new_n243), .c(new_n242), .d(new_n235), .o1(new_n244));
  tech160nm_fixnrc02aa1n02p5x5 g149(.a(\b[23] ), .b(\a[24] ), .out0(new_n245));
  tech160nm_fiaoi012aa1n02p5x5 g150(.a(new_n245), .b(new_n244), .c(new_n241), .o1(new_n246));
  tech160nm_fiaoi012aa1n04x5   g151(.a(new_n238), .b(new_n234), .c(new_n237), .o1(new_n247));
  nano22aa1n03x7               g152(.a(new_n247), .b(new_n241), .c(new_n245), .out0(new_n248));
  norp02aa1n03x5               g153(.a(new_n246), .b(new_n248), .o1(\s[24] ));
  nor002aa1n03x5               g154(.a(new_n245), .b(new_n238), .o1(new_n250));
  nand02aa1d04x5               g155(.a(new_n250), .b(new_n235), .o1(new_n251));
  nona22aa1n03x5               g156(.a(new_n194), .b(new_n233), .c(new_n251), .out0(new_n252));
  nano23aa1n06x5               g157(.a(new_n201), .b(new_n214), .c(new_n215), .d(new_n202), .out0(new_n253));
  aoai13aa1n06x5               g158(.a(new_n235), .b(new_n222), .c(new_n253), .d(new_n208), .o1(new_n254));
  inv020aa1n04x5               g159(.a(new_n236), .o1(new_n255));
  inv020aa1n03x5               g160(.a(new_n250), .o1(new_n256));
  oaoi03aa1n02x5               g161(.a(\a[24] ), .b(\b[23] ), .c(new_n241), .o1(new_n257));
  inv020aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  aoai13aa1n12x5               g163(.a(new_n258), .b(new_n256), .c(new_n254), .d(new_n255), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  xnrc02aa1n12x5               g165(.a(\b[24] ), .b(\a[25] ), .out0(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  xnbna2aa1n03x5               g167(.a(new_n262), .b(new_n252), .c(new_n260), .out0(\s[25] ));
  nor042aa1n03x5               g168(.a(\b[24] ), .b(\a[25] ), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n251), .o1(new_n266));
  aoai13aa1n03x5               g171(.a(new_n262), .b(new_n259), .c(new_n242), .d(new_n266), .o1(new_n267));
  xnrc02aa1n02x5               g172(.a(\b[25] ), .b(\a[26] ), .out0(new_n268));
  tech160nm_fiaoi012aa1n02p5x5 g173(.a(new_n268), .b(new_n267), .c(new_n265), .o1(new_n269));
  aoi012aa1n03x5               g174(.a(new_n261), .b(new_n252), .c(new_n260), .o1(new_n270));
  nano22aa1n03x5               g175(.a(new_n270), .b(new_n265), .c(new_n268), .out0(new_n271));
  norp02aa1n03x5               g176(.a(new_n269), .b(new_n271), .o1(\s[26] ));
  nor042aa1n03x5               g177(.a(new_n268), .b(new_n261), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  nona32aa1n09x5               g179(.a(new_n194), .b(new_n274), .c(new_n251), .d(new_n233), .out0(new_n275));
  oao003aa1n02x5               g180(.a(\a[26] ), .b(\b[25] ), .c(new_n265), .carry(new_n276));
  aobi12aa1n12x5               g181(.a(new_n276), .b(new_n259), .c(new_n273), .out0(new_n277));
  nor042aa1n03x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  and002aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o(new_n279));
  norp02aa1n02x5               g184(.a(new_n279), .b(new_n278), .o1(new_n280));
  xnbna2aa1n03x5               g185(.a(new_n280), .b(new_n277), .c(new_n275), .out0(\s[27] ));
  inv000aa1d42x5               g186(.a(new_n279), .o1(new_n282));
  xorc02aa1n02x5               g187(.a(\a[28] ), .b(\b[27] ), .out0(new_n283));
  inv000aa1d42x5               g188(.a(new_n278), .o1(new_n284));
  nanp03aa1n03x5               g189(.a(new_n277), .b(new_n275), .c(new_n284), .o1(new_n285));
  nanp03aa1n03x5               g190(.a(new_n285), .b(new_n282), .c(new_n283), .o1(new_n286));
  aoi012aa1n02x7               g191(.a(new_n283), .b(new_n285), .c(new_n282), .o1(new_n287));
  norb02aa1n03x4               g192(.a(new_n286), .b(new_n287), .out0(\s[28] ));
  nano22aa1n02x4               g193(.a(new_n274), .b(new_n235), .c(new_n250), .out0(new_n289));
  aoai13aa1n02x5               g194(.a(new_n250), .b(new_n236), .c(new_n224), .d(new_n235), .o1(new_n290));
  aoai13aa1n03x5               g195(.a(new_n276), .b(new_n274), .c(new_n290), .d(new_n258), .o1(new_n291));
  and002aa1n02x5               g196(.a(new_n283), .b(new_n280), .o(new_n292));
  aoai13aa1n02x5               g197(.a(new_n292), .b(new_n291), .c(new_n242), .d(new_n289), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[28] ), .b(\a[29] ), .out0(new_n295));
  tech160nm_fiaoi012aa1n02p5x5 g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  aobi12aa1n02x7               g201(.a(new_n292), .b(new_n277), .c(new_n275), .out0(new_n297));
  nano22aa1n03x5               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n158), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g205(.a(new_n295), .b(new_n283), .c(new_n280), .out0(new_n301));
  aoai13aa1n02x5               g206(.a(new_n301), .b(new_n291), .c(new_n242), .d(new_n289), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[29] ), .b(\a[30] ), .out0(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n304), .b(new_n302), .c(new_n303), .o1(new_n305));
  aobi12aa1n02x7               g210(.a(new_n301), .b(new_n277), .c(new_n275), .out0(new_n306));
  nano22aa1n03x5               g211(.a(new_n306), .b(new_n303), .c(new_n304), .out0(new_n307));
  norp02aa1n03x5               g212(.a(new_n305), .b(new_n307), .o1(\s[30] ));
  nano23aa1n02x4               g213(.a(new_n304), .b(new_n295), .c(new_n283), .d(new_n280), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n309), .b(new_n291), .c(new_n242), .d(new_n289), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n303), .carry(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  tech160nm_fiaoi012aa1n02p5x5 g217(.a(new_n312), .b(new_n310), .c(new_n311), .o1(new_n313));
  aobi12aa1n02x7               g218(.a(new_n309), .b(new_n277), .c(new_n275), .out0(new_n314));
  nano22aa1n03x5               g219(.a(new_n314), .b(new_n311), .c(new_n312), .out0(new_n315));
  norp02aa1n03x5               g220(.a(new_n313), .b(new_n315), .o1(\s[31] ));
  xnbna2aa1n03x5               g221(.a(new_n156), .b(new_n105), .c(new_n104), .out0(\s[3] ));
  nanp02aa1n02x5               g222(.a(new_n161), .b(new_n113), .o1(new_n318));
  obai22aa1n02x7               g223(.a(new_n113), .b(new_n106), .c(new_n159), .d(new_n156), .out0(new_n319));
  oa0022aa1n02x5               g224(.a(new_n318), .b(new_n106), .c(new_n319), .d(new_n102), .o(\s[4] ));
  xnbna2aa1n03x5               g225(.a(new_n162), .b(new_n161), .c(new_n113), .out0(\s[5] ));
  oao003aa1n02x5               g226(.a(\a[5] ), .b(\b[4] ), .c(new_n318), .carry(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g228(.a(\a[6] ), .b(\b[5] ), .c(new_n322), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g230(.a(new_n123), .b(new_n124), .c(new_n324), .o1(new_n326));
  xnrb03aa1n02x5               g231(.a(new_n326), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g232(.a(new_n165), .b(\b[8] ), .c(new_n100), .out0(\s[9] ));
endmodule


