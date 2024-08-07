// Benchmark "adder" written by ABC on Thu Jul 18 10:11:21 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n322, new_n324;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n06x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nand22aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  aoi112aa1n03x5               g005(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n101));
  nor002aa1n02x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  oaih22aa1d12x5               g007(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n103));
  xnrc02aa1n12x5               g008(.a(\b[7] ), .b(\a[8] ), .out0(new_n104));
  nor002aa1n20x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nand02aa1d24x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  nanb02aa1n02x5               g011(.a(new_n105), .b(new_n106), .out0(new_n107));
  nand42aa1d28x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nano23aa1n03x5               g013(.a(new_n104), .b(new_n107), .c(new_n108), .d(new_n103), .out0(new_n109));
  norp03aa1n06x5               g014(.a(new_n109), .b(new_n102), .c(new_n101), .o1(new_n110));
  nor042aa1n02x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  nand22aa1n06x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nand02aa1d04x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  aoi012aa1n06x5               g018(.a(new_n111), .b(new_n112), .c(new_n113), .o1(new_n114));
  norp02aa1n09x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  nanp02aa1n04x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nor022aa1n16x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nand42aa1n03x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nona23aa1n09x5               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  tech160nm_fiaoi012aa1n03p5x5 g024(.a(new_n115), .b(new_n117), .c(new_n116), .o1(new_n120));
  oai012aa1n12x5               g025(.a(new_n120), .b(new_n119), .c(new_n114), .o1(new_n121));
  norb02aa1n12x5               g026(.a(new_n106), .b(new_n105), .out0(new_n122));
  nor042aa1d18x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  norb02aa1n02x7               g028(.a(new_n108), .b(new_n123), .out0(new_n124));
  xnrc02aa1n02x5               g029(.a(\b[4] ), .b(\a[5] ), .out0(new_n125));
  nano23aa1n06x5               g030(.a(new_n125), .b(new_n104), .c(new_n122), .d(new_n124), .out0(new_n126));
  nand22aa1n03x5               g031(.a(new_n126), .b(new_n121), .o1(new_n127));
  nand02aa1d06x5               g032(.a(new_n127), .b(new_n110), .o1(new_n128));
  aoai13aa1n02x5               g033(.a(new_n98), .b(new_n99), .c(new_n128), .d(new_n100), .o1(new_n129));
  inv020aa1d32x5               g034(.a(\a[5] ), .o1(new_n130));
  inv040aa1d30x5               g035(.a(\b[4] ), .o1(new_n131));
  tech160nm_fiaoi012aa1n05x5   g036(.a(new_n123), .b(new_n130), .c(new_n131), .o1(new_n132));
  xorc02aa1n03x5               g037(.a(\a[8] ), .b(\b[7] ), .out0(new_n133));
  inv030aa1n02x5               g038(.a(new_n108), .o1(new_n134));
  nona23aa1n06x5               g039(.a(new_n133), .b(new_n122), .c(new_n132), .d(new_n134), .out0(new_n135));
  nona22aa1d24x5               g040(.a(new_n135), .b(new_n102), .c(new_n101), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n100), .b(new_n136), .c(new_n126), .d(new_n121), .o1(new_n137));
  nona22aa1n03x5               g042(.a(new_n137), .b(new_n99), .c(new_n98), .out0(new_n138));
  nanp02aa1n02x5               g043(.a(new_n129), .b(new_n138), .o1(\s[10] ));
  nand42aa1n04x5               g044(.a(\b[9] ), .b(\a[10] ), .o1(new_n140));
  nor022aa1n16x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand02aa1n06x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  nanb02aa1n06x5               g047(.a(new_n141), .b(new_n142), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n138), .c(new_n140), .out0(\s[11] ));
  aoi013aa1n02x4               g049(.a(new_n141), .b(new_n138), .c(new_n140), .d(new_n142), .o1(new_n145));
  nor042aa1n06x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nanp02aa1n04x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n06x4               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  xnrc02aa1n02x5               g053(.a(new_n145), .b(new_n148), .out0(\s[12] ));
  norb02aa1n06x5               g054(.a(new_n100), .b(new_n99), .out0(new_n150));
  nano23aa1n06x5               g055(.a(new_n141), .b(new_n146), .c(new_n147), .d(new_n142), .out0(new_n151));
  nand23aa1d12x5               g056(.a(new_n151), .b(new_n97), .c(new_n150), .o1(new_n152));
  aoi112aa1n06x5               g057(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n153));
  nanb02aa1n03x5               g058(.a(new_n146), .b(new_n147), .out0(new_n154));
  oai022aa1n02x5               g059(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n155));
  nano23aa1n06x5               g060(.a(new_n154), .b(new_n143), .c(new_n155), .d(new_n140), .out0(new_n156));
  nor043aa1n03x5               g061(.a(new_n156), .b(new_n153), .c(new_n146), .o1(new_n157));
  aoai13aa1n06x5               g062(.a(new_n157), .b(new_n152), .c(new_n127), .d(new_n110), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n10x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nand42aa1d28x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  aoi012aa1n03x5               g066(.a(new_n160), .b(new_n158), .c(new_n161), .o1(new_n162));
  xnrb03aa1n03x5               g067(.a(new_n162), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv000aa1n02x5               g068(.a(new_n152), .o1(new_n164));
  aoai13aa1n03x5               g069(.a(new_n164), .b(new_n136), .c(new_n126), .d(new_n121), .o1(new_n165));
  nor002aa1n10x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nanp02aa1n24x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nano23aa1d15x5               g072(.a(new_n160), .b(new_n166), .c(new_n167), .d(new_n161), .out0(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  oai012aa1n04x7               g074(.a(new_n167), .b(new_n166), .c(new_n160), .o1(new_n170));
  aoai13aa1n04x5               g075(.a(new_n170), .b(new_n169), .c(new_n165), .d(new_n157), .o1(new_n171));
  xorb03aa1n02x5               g076(.a(new_n171), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nand22aa1n09x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  norb02aa1n03x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  nor002aa1n12x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nand02aa1n03x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  norb02aa1n06x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  aoi112aa1n02x5               g083(.a(new_n178), .b(new_n173), .c(new_n171), .d(new_n175), .o1(new_n179));
  aoai13aa1n03x5               g084(.a(new_n178), .b(new_n173), .c(new_n171), .d(new_n174), .o1(new_n180));
  norb02aa1n02x7               g085(.a(new_n180), .b(new_n179), .out0(\s[16] ));
  inv000aa1n02x5               g086(.a(new_n140), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n142), .b(new_n141), .out0(new_n183));
  oab012aa1n03x5               g088(.a(new_n99), .b(\a[10] ), .c(\b[9] ), .out0(new_n184));
  nona23aa1n03x5               g089(.a(new_n148), .b(new_n183), .c(new_n184), .d(new_n182), .out0(new_n185));
  nona22aa1n02x4               g090(.a(new_n185), .b(new_n153), .c(new_n146), .out0(new_n186));
  nand23aa1d12x5               g091(.a(new_n168), .b(new_n175), .c(new_n178), .o1(new_n187));
  inv000aa1n03x5               g092(.a(new_n187), .o1(new_n188));
  nona23aa1n02x5               g093(.a(new_n177), .b(new_n174), .c(new_n173), .d(new_n176), .out0(new_n189));
  nanp02aa1n02x5               g094(.a(new_n173), .b(new_n177), .o1(new_n190));
  oai122aa1n06x5               g095(.a(new_n190), .b(new_n189), .c(new_n170), .d(\b[15] ), .e(\a[16] ), .o1(new_n191));
  aoi012aa1n12x5               g096(.a(new_n191), .b(new_n186), .c(new_n188), .o1(new_n192));
  nor042aa1n06x5               g097(.a(new_n187), .b(new_n152), .o1(new_n193));
  aoai13aa1n12x5               g098(.a(new_n193), .b(new_n136), .c(new_n126), .d(new_n121), .o1(new_n194));
  xorc02aa1n12x5               g099(.a(\a[17] ), .b(\b[16] ), .out0(new_n195));
  xnbna2aa1n03x5               g100(.a(new_n195), .b(new_n194), .c(new_n192), .out0(\s[17] ));
  inv040aa1d32x5               g101(.a(\a[17] ), .o1(new_n197));
  inv040aa1n12x5               g102(.a(\b[16] ), .o1(new_n198));
  nand22aa1n03x5               g103(.a(new_n198), .b(new_n197), .o1(new_n199));
  oabi12aa1n03x5               g104(.a(new_n191), .b(new_n157), .c(new_n187), .out0(new_n200));
  aoai13aa1n02x5               g105(.a(new_n195), .b(new_n200), .c(new_n128), .d(new_n193), .o1(new_n201));
  nor002aa1d32x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nand02aa1d20x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n204), .b(new_n201), .c(new_n199), .out0(\s[18] ));
  nanp02aa1n02x5               g110(.a(\b[16] ), .b(\a[17] ), .o1(new_n206));
  nano32aa1d12x5               g111(.a(new_n202), .b(new_n199), .c(new_n203), .d(new_n206), .out0(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n12x5               g113(.a(new_n203), .b(new_n202), .c(new_n197), .d(new_n198), .o1(new_n209));
  aoai13aa1n04x5               g114(.a(new_n209), .b(new_n208), .c(new_n194), .d(new_n192), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nand22aa1n12x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nor002aa1d32x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nand22aa1n12x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  aoi112aa1n03x4               g122(.a(new_n213), .b(new_n217), .c(new_n210), .d(new_n214), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n217), .b(new_n213), .c(new_n210), .d(new_n214), .o1(new_n219));
  norb02aa1n02x7               g124(.a(new_n219), .b(new_n218), .out0(\s[20] ));
  nano23aa1n06x5               g125(.a(new_n213), .b(new_n215), .c(new_n216), .d(new_n214), .out0(new_n221));
  nanp03aa1n02x5               g126(.a(new_n221), .b(new_n195), .c(new_n204), .o1(new_n222));
  nona23aa1n09x5               g127(.a(new_n216), .b(new_n214), .c(new_n213), .d(new_n215), .out0(new_n223));
  tech160nm_fiaoi012aa1n03p5x5 g128(.a(new_n215), .b(new_n213), .c(new_n216), .o1(new_n224));
  oai012aa1n12x5               g129(.a(new_n224), .b(new_n223), .c(new_n209), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n04x5               g131(.a(new_n226), .b(new_n222), .c(new_n194), .d(new_n192), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n06x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[21] ), .b(\b[20] ), .out0(new_n230));
  xorc02aa1n02x5               g135(.a(\a[22] ), .b(\b[21] ), .out0(new_n231));
  aoi112aa1n02x5               g136(.a(new_n229), .b(new_n231), .c(new_n227), .d(new_n230), .o1(new_n232));
  aoai13aa1n03x5               g137(.a(new_n231), .b(new_n229), .c(new_n227), .d(new_n230), .o1(new_n233));
  norb02aa1n02x7               g138(.a(new_n233), .b(new_n232), .out0(\s[22] ));
  inv000aa1d42x5               g139(.a(\a[21] ), .o1(new_n235));
  inv040aa1d32x5               g140(.a(\a[22] ), .o1(new_n236));
  xroi22aa1d06x4               g141(.a(new_n235), .b(\b[20] ), .c(new_n236), .d(\b[21] ), .out0(new_n237));
  nand23aa1n03x5               g142(.a(new_n237), .b(new_n207), .c(new_n221), .o1(new_n238));
  inv040aa1n02x5               g143(.a(new_n209), .o1(new_n239));
  inv020aa1n02x5               g144(.a(new_n224), .o1(new_n240));
  aoai13aa1n06x5               g145(.a(new_n237), .b(new_n240), .c(new_n221), .d(new_n239), .o1(new_n241));
  inv000aa1d42x5               g146(.a(\b[21] ), .o1(new_n242));
  oaoi03aa1n09x5               g147(.a(new_n236), .b(new_n242), .c(new_n229), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(new_n241), .b(new_n243), .o1(new_n244));
  inv000aa1n02x5               g149(.a(new_n244), .o1(new_n245));
  aoai13aa1n04x5               g150(.a(new_n245), .b(new_n238), .c(new_n194), .d(new_n192), .o1(new_n246));
  xorb03aa1n02x5               g151(.a(new_n246), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  xorc02aa1n12x5               g153(.a(\a[23] ), .b(\b[22] ), .out0(new_n249));
  tech160nm_fixorc02aa1n05x5   g154(.a(\a[24] ), .b(\b[23] ), .out0(new_n250));
  aoi112aa1n02x5               g155(.a(new_n248), .b(new_n250), .c(new_n246), .d(new_n249), .o1(new_n251));
  aoai13aa1n03x5               g156(.a(new_n250), .b(new_n248), .c(new_n246), .d(new_n249), .o1(new_n252));
  norb02aa1n02x7               g157(.a(new_n252), .b(new_n251), .out0(\s[24] ));
  and002aa1n02x5               g158(.a(new_n250), .b(new_n249), .o(new_n254));
  nanb03aa1n02x5               g159(.a(new_n222), .b(new_n254), .c(new_n237), .out0(new_n255));
  inv030aa1n02x5               g160(.a(new_n254), .o1(new_n256));
  orn002aa1n02x5               g161(.a(\a[23] ), .b(\b[22] ), .o(new_n257));
  oao003aa1n02x5               g162(.a(\a[24] ), .b(\b[23] ), .c(new_n257), .carry(new_n258));
  aoai13aa1n04x5               g163(.a(new_n258), .b(new_n256), .c(new_n241), .d(new_n243), .o1(new_n259));
  inv040aa1n03x5               g164(.a(new_n259), .o1(new_n260));
  aoai13aa1n04x5               g165(.a(new_n260), .b(new_n255), .c(new_n194), .d(new_n192), .o1(new_n261));
  xorb03aa1n02x5               g166(.a(new_n261), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  tech160nm_fixorc02aa1n05x5   g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  tech160nm_fixorc02aa1n05x5   g169(.a(\a[26] ), .b(\b[25] ), .out0(new_n265));
  aoi112aa1n02x5               g170(.a(new_n263), .b(new_n265), .c(new_n261), .d(new_n264), .o1(new_n266));
  aoai13aa1n03x5               g171(.a(new_n265), .b(new_n263), .c(new_n261), .d(new_n264), .o1(new_n267));
  norb02aa1n02x7               g172(.a(new_n267), .b(new_n266), .out0(\s[26] ));
  and002aa1n09x5               g173(.a(new_n265), .b(new_n264), .o(new_n269));
  nano22aa1n03x7               g174(.a(new_n238), .b(new_n254), .c(new_n269), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n200), .c(new_n128), .d(new_n193), .o1(new_n271));
  orn002aa1n02x5               g176(.a(\a[25] ), .b(\b[24] ), .o(new_n272));
  oao003aa1n02x5               g177(.a(\a[26] ), .b(\b[25] ), .c(new_n272), .carry(new_n273));
  aobi12aa1n06x5               g178(.a(new_n273), .b(new_n259), .c(new_n269), .out0(new_n274));
  xorc02aa1n12x5               g179(.a(\a[27] ), .b(\b[26] ), .out0(new_n275));
  xnbna2aa1n03x5               g180(.a(new_n275), .b(new_n274), .c(new_n271), .out0(\s[27] ));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  inv040aa1n03x5               g182(.a(new_n277), .o1(new_n278));
  aobi12aa1n02x7               g183(.a(new_n275), .b(new_n274), .c(new_n271), .out0(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  nano22aa1n02x4               g185(.a(new_n279), .b(new_n278), .c(new_n280), .out0(new_n281));
  inv030aa1n02x5               g186(.a(new_n270), .o1(new_n282));
  aoi012aa1n06x5               g187(.a(new_n282), .b(new_n194), .c(new_n192), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n243), .o1(new_n284));
  aoai13aa1n02x7               g189(.a(new_n254), .b(new_n284), .c(new_n225), .d(new_n237), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n269), .o1(new_n286));
  aoai13aa1n06x5               g191(.a(new_n273), .b(new_n286), .c(new_n285), .d(new_n258), .o1(new_n287));
  oaih12aa1n02x5               g192(.a(new_n275), .b(new_n287), .c(new_n283), .o1(new_n288));
  tech160nm_fiaoi012aa1n03p5x5 g193(.a(new_n280), .b(new_n288), .c(new_n278), .o1(new_n289));
  nor002aa1n02x5               g194(.a(new_n289), .b(new_n281), .o1(\s[28] ));
  norb02aa1n02x5               g195(.a(new_n275), .b(new_n280), .out0(new_n291));
  oaih12aa1n02x5               g196(.a(new_n291), .b(new_n287), .c(new_n283), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  tech160nm_fiaoi012aa1n03p5x5 g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  aobi12aa1n02x7               g200(.a(new_n291), .b(new_n274), .c(new_n271), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n296), .b(new_n293), .c(new_n294), .out0(new_n297));
  nor002aa1n02x5               g202(.a(new_n295), .b(new_n297), .o1(\s[29] ));
  xorb03aa1n02x5               g203(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g204(.a(new_n275), .b(new_n294), .c(new_n280), .out0(new_n300));
  oaih12aa1n02x5               g205(.a(new_n300), .b(new_n287), .c(new_n283), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[29] ), .b(\a[30] ), .out0(new_n303));
  tech160nm_fiaoi012aa1n03p5x5 g208(.a(new_n303), .b(new_n301), .c(new_n302), .o1(new_n304));
  aobi12aa1n02x7               g209(.a(new_n300), .b(new_n274), .c(new_n271), .out0(new_n305));
  nano22aa1n02x4               g210(.a(new_n305), .b(new_n302), .c(new_n303), .out0(new_n306));
  nor002aa1n02x5               g211(.a(new_n304), .b(new_n306), .o1(\s[30] ));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  norb02aa1n02x7               g213(.a(new_n300), .b(new_n303), .out0(new_n309));
  aobi12aa1n02x7               g214(.a(new_n309), .b(new_n274), .c(new_n271), .out0(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n311));
  nano22aa1n02x4               g216(.a(new_n310), .b(new_n308), .c(new_n311), .out0(new_n312));
  oaih12aa1n02x5               g217(.a(new_n309), .b(new_n287), .c(new_n283), .o1(new_n313));
  tech160nm_fiaoi012aa1n03p5x5 g218(.a(new_n308), .b(new_n313), .c(new_n311), .o1(new_n314));
  nor002aa1n02x5               g219(.a(new_n314), .b(new_n312), .o1(\s[31] ));
  xnrb03aa1n02x5               g220(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g221(.a(\a[3] ), .b(\b[2] ), .c(new_n114), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g223(.a(new_n121), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oao003aa1n03x5               g224(.a(new_n130), .b(new_n131), .c(new_n121), .carry(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fioai012aa1n05x5   g226(.a(new_n108), .b(new_n320), .c(new_n123), .o1(new_n322));
  xnrc02aa1n02x5               g227(.a(new_n322), .b(new_n122), .out0(\s[7] ));
  oaoi03aa1n03x5               g228(.a(\a[7] ), .b(\b[6] ), .c(new_n322), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g230(.a(new_n150), .b(new_n127), .c(new_n110), .out0(\s[9] ));
endmodule


