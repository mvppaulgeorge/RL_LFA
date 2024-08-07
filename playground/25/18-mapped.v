// Benchmark "adder" written by ABC on Thu Jul 18 00:54:58 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n290, new_n291, new_n292, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n346, new_n349, new_n351, new_n352, new_n354;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  tech160nm_finor002aa1n05x5   g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n04x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand22aa1n04x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n12x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  xorc02aa1n02x5               g007(.a(\a[4] ), .b(\b[3] ), .out0(new_n103));
  nor042aa1n03x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n02x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nanb03aa1n06x5               g011(.a(new_n102), .b(new_n103), .c(new_n106), .out0(new_n107));
  inv000aa1d42x5               g012(.a(\a[4] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\b[3] ), .o1(new_n109));
  oaoi03aa1n09x5               g014(.a(new_n108), .b(new_n109), .c(new_n104), .o1(new_n110));
  nor042aa1n02x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nand42aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nanb02aa1n06x5               g017(.a(new_n111), .b(new_n112), .out0(new_n113));
  tech160nm_fixnrc02aa1n04x5   g018(.a(\b[4] ), .b(\a[5] ), .out0(new_n114));
  norp02aa1n04x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nand22aa1n03x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nor002aa1n06x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nano23aa1n06x5               g023(.a(new_n115), .b(new_n117), .c(new_n118), .d(new_n116), .out0(new_n119));
  nona22aa1n02x4               g024(.a(new_n119), .b(new_n114), .c(new_n113), .out0(new_n120));
  tech160nm_fiao0012aa1n02p5x5 g025(.a(new_n115), .b(new_n117), .c(new_n116), .o(new_n121));
  inv000aa1d42x5               g026(.a(\a[5] ), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\b[4] ), .o1(new_n123));
  aoai13aa1n06x5               g028(.a(new_n112), .b(new_n111), .c(new_n122), .d(new_n123), .o1(new_n124));
  aoib12aa1n06x5               g029(.a(new_n121), .b(new_n119), .c(new_n124), .out0(new_n125));
  aoai13aa1n06x5               g030(.a(new_n125), .b(new_n120), .c(new_n107), .d(new_n110), .o1(new_n126));
  oaoi03aa1n02x5               g031(.a(new_n97), .b(new_n98), .c(new_n126), .o1(new_n127));
  xnrb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n12x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand42aa1n04x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norp02aa1n02x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  nand42aa1n03x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  nano23aa1n06x5               g037(.a(new_n129), .b(new_n131), .c(new_n132), .d(new_n130), .out0(new_n133));
  aoai13aa1n04x5               g038(.a(new_n130), .b(new_n129), .c(new_n97), .d(new_n98), .o1(new_n134));
  inv040aa1n02x5               g039(.a(new_n134), .o1(new_n135));
  nor002aa1n06x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand42aa1n06x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  aoai13aa1n03x5               g043(.a(new_n138), .b(new_n135), .c(new_n126), .d(new_n133), .o1(new_n139));
  aoi112aa1n02x5               g044(.a(new_n138), .b(new_n135), .c(new_n126), .d(new_n133), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n139), .b(new_n140), .out0(\s[11] ));
  oai012aa1n02x5               g046(.a(new_n139), .b(\b[10] ), .c(\a[11] ), .o1(new_n142));
  xorb03aa1n02x5               g047(.a(new_n142), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nanp02aa1n02x5               g048(.a(new_n109), .b(new_n108), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(\b[3] ), .b(\a[4] ), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(new_n144), .b(new_n145), .o1(new_n146));
  inv000aa1d42x5               g051(.a(\a[3] ), .o1(new_n147));
  inv000aa1d42x5               g052(.a(\b[2] ), .o1(new_n148));
  nand42aa1n03x5               g053(.a(new_n148), .b(new_n147), .o1(new_n149));
  nand42aa1n02x5               g054(.a(new_n149), .b(new_n105), .o1(new_n150));
  norp03aa1n03x5               g055(.a(new_n102), .b(new_n146), .c(new_n150), .o1(new_n151));
  inv000aa1n02x5               g056(.a(new_n110), .o1(new_n152));
  nona23aa1n03x5               g057(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n153));
  nor043aa1n03x5               g058(.a(new_n153), .b(new_n114), .c(new_n113), .o1(new_n154));
  tech160nm_fioai012aa1n03p5x5 g059(.a(new_n154), .b(new_n151), .c(new_n152), .o1(new_n155));
  nor022aa1n06x5               g060(.a(\b[11] ), .b(\a[12] ), .o1(new_n156));
  nand02aa1d04x5               g061(.a(\b[11] ), .b(\a[12] ), .o1(new_n157));
  nano23aa1n06x5               g062(.a(new_n136), .b(new_n156), .c(new_n157), .d(new_n137), .out0(new_n158));
  nand02aa1d08x5               g063(.a(new_n158), .b(new_n133), .o1(new_n159));
  aoi012aa1n02x7               g064(.a(new_n156), .b(new_n136), .c(new_n157), .o1(new_n160));
  aobi12aa1n03x5               g065(.a(new_n160), .b(new_n158), .c(new_n135), .out0(new_n161));
  aoai13aa1n03x5               g066(.a(new_n161), .b(new_n159), .c(new_n155), .d(new_n125), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g068(.a(\a[13] ), .o1(new_n164));
  inv030aa1d32x5               g069(.a(\b[12] ), .o1(new_n165));
  oaoi03aa1n02x5               g070(.a(new_n164), .b(new_n165), .c(new_n162), .o1(new_n166));
  xnrb03aa1n03x5               g071(.a(new_n166), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oai013aa1n06x5               g072(.a(new_n110), .b(new_n102), .c(new_n146), .d(new_n150), .o1(new_n168));
  oabi12aa1n02x5               g073(.a(new_n121), .b(new_n153), .c(new_n124), .out0(new_n169));
  inv000aa1d42x5               g074(.a(new_n159), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n169), .c(new_n168), .d(new_n154), .o1(new_n171));
  nor002aa1d32x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nand42aa1n08x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  aoai13aa1n12x5               g078(.a(new_n173), .b(new_n172), .c(new_n164), .d(new_n165), .o1(new_n174));
  nor022aa1n04x5               g079(.a(\b[12] ), .b(\a[13] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(\b[12] ), .b(\a[13] ), .o1(new_n176));
  nona23aa1n02x4               g081(.a(new_n173), .b(new_n176), .c(new_n175), .d(new_n172), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n174), .b(new_n177), .c(new_n171), .d(new_n161), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n180), .o1(new_n181));
  nand02aa1n06x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  nanb02aa1n18x5               g087(.a(new_n180), .b(new_n182), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n174), .o1(new_n185));
  nano23aa1n06x5               g090(.a(new_n175), .b(new_n172), .c(new_n173), .d(new_n176), .out0(new_n186));
  aoai13aa1n03x5               g091(.a(new_n184), .b(new_n185), .c(new_n162), .d(new_n186), .o1(new_n187));
  nor042aa1n06x5               g092(.a(\b[15] ), .b(\a[16] ), .o1(new_n188));
  nanp02aa1n04x5               g093(.a(\b[15] ), .b(\a[16] ), .o1(new_n189));
  nanb02aa1n12x5               g094(.a(new_n188), .b(new_n189), .out0(new_n190));
  tech160nm_fiaoi012aa1n02p5x5 g095(.a(new_n190), .b(new_n187), .c(new_n181), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n190), .o1(new_n192));
  aoi112aa1n02x5               g097(.a(new_n192), .b(new_n180), .c(new_n178), .d(new_n184), .o1(new_n193));
  norp02aa1n03x5               g098(.a(new_n191), .b(new_n193), .o1(\s[16] ));
  nona22aa1n09x5               g099(.a(new_n186), .b(new_n190), .c(new_n183), .out0(new_n195));
  nor042aa1n06x5               g100(.a(new_n195), .b(new_n159), .o1(new_n196));
  aoai13aa1n09x5               g101(.a(new_n196), .b(new_n169), .c(new_n168), .d(new_n154), .o1(new_n197));
  nona23aa1n03x5               g102(.a(new_n157), .b(new_n137), .c(new_n136), .d(new_n156), .out0(new_n198));
  oai012aa1n03x5               g103(.a(new_n160), .b(new_n198), .c(new_n134), .o1(new_n199));
  nona23aa1n03x5               g104(.a(new_n189), .b(new_n182), .c(new_n180), .d(new_n188), .out0(new_n200));
  nor042aa1n02x5               g105(.a(new_n200), .b(new_n177), .o1(new_n201));
  aoi012aa1n02x7               g106(.a(new_n188), .b(new_n180), .c(new_n189), .o1(new_n202));
  oai012aa1n06x5               g107(.a(new_n202), .b(new_n200), .c(new_n174), .o1(new_n203));
  aoi012aa1n12x5               g108(.a(new_n203), .b(new_n199), .c(new_n201), .o1(new_n204));
  xorc02aa1n12x5               g109(.a(\a[17] ), .b(\b[16] ), .out0(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n197), .c(new_n204), .out0(\s[17] ));
  inv040aa1d32x5               g111(.a(\a[17] ), .o1(new_n207));
  inv040aa1d32x5               g112(.a(\b[16] ), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(new_n208), .b(new_n207), .o1(new_n209));
  oabi12aa1n03x5               g114(.a(new_n203), .b(new_n161), .c(new_n195), .out0(new_n210));
  aoai13aa1n02x5               g115(.a(new_n205), .b(new_n210), .c(new_n126), .d(new_n196), .o1(new_n211));
  nor002aa1d32x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  nand42aa1d28x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  nanb02aa1n12x5               g118(.a(new_n212), .b(new_n213), .out0(new_n214));
  xobna2aa1n03x5               g119(.a(new_n214), .b(new_n211), .c(new_n209), .out0(\s[18] ));
  norb02aa1n09x5               g120(.a(new_n205), .b(new_n214), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  aoai13aa1n12x5               g122(.a(new_n213), .b(new_n212), .c(new_n207), .d(new_n208), .o1(new_n218));
  aoai13aa1n04x5               g123(.a(new_n218), .b(new_n217), .c(new_n197), .d(new_n204), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  nona23aa1n02x5               g128(.a(new_n186), .b(new_n133), .c(new_n200), .d(new_n198), .out0(new_n224));
  aoai13aa1n06x5               g129(.a(new_n204), .b(new_n224), .c(new_n155), .d(new_n125), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n218), .o1(new_n226));
  nand22aa1n12x5               g131(.a(\b[18] ), .b(\a[19] ), .o1(new_n227));
  nanb02aa1d24x5               g132(.a(new_n222), .b(new_n227), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n03x5               g134(.a(new_n229), .b(new_n226), .c(new_n225), .d(new_n216), .o1(new_n230));
  nor042aa1n06x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  nand02aa1n06x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  nanb02aa1n03x5               g137(.a(new_n231), .b(new_n232), .out0(new_n233));
  tech160nm_fiaoi012aa1n02p5x5 g138(.a(new_n233), .b(new_n230), .c(new_n223), .o1(new_n234));
  inv040aa1n02x5               g139(.a(new_n233), .o1(new_n235));
  aoi112aa1n03x4               g140(.a(new_n222), .b(new_n235), .c(new_n219), .d(new_n229), .o1(new_n236));
  norp02aa1n03x5               g141(.a(new_n234), .b(new_n236), .o1(\s[20] ));
  nona23aa1d24x5               g142(.a(new_n205), .b(new_n235), .c(new_n228), .d(new_n214), .out0(new_n238));
  nona23aa1n06x5               g143(.a(new_n232), .b(new_n227), .c(new_n222), .d(new_n231), .out0(new_n239));
  tech160nm_fiaoi012aa1n03p5x5 g144(.a(new_n231), .b(new_n222), .c(new_n232), .o1(new_n240));
  oai012aa1d24x5               g145(.a(new_n240), .b(new_n239), .c(new_n218), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  aoai13aa1n04x5               g147(.a(new_n242), .b(new_n238), .c(new_n197), .d(new_n204), .o1(new_n243));
  xorb03aa1n02x5               g148(.a(new_n243), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g149(.a(\b[20] ), .b(\a[21] ), .o1(new_n245));
  inv040aa1n08x5               g150(.a(new_n245), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n238), .o1(new_n247));
  nanp02aa1n09x5               g152(.a(\b[20] ), .b(\a[21] ), .o1(new_n248));
  nanb02aa1n02x5               g153(.a(new_n245), .b(new_n248), .out0(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  aoai13aa1n03x5               g155(.a(new_n250), .b(new_n241), .c(new_n225), .d(new_n247), .o1(new_n251));
  nor042aa1d18x5               g156(.a(\b[21] ), .b(\a[22] ), .o1(new_n252));
  nand42aa1n04x5               g157(.a(\b[21] ), .b(\a[22] ), .o1(new_n253));
  nanb02aa1n02x5               g158(.a(new_n252), .b(new_n253), .out0(new_n254));
  aoi012aa1n03x5               g159(.a(new_n254), .b(new_n251), .c(new_n246), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n254), .o1(new_n256));
  aoi112aa1n03x4               g161(.a(new_n245), .b(new_n256), .c(new_n243), .d(new_n250), .o1(new_n257));
  norp02aa1n03x5               g162(.a(new_n255), .b(new_n257), .o1(\s[22] ));
  nano23aa1n02x4               g163(.a(new_n222), .b(new_n231), .c(new_n232), .d(new_n227), .out0(new_n259));
  nona23aa1n02x5               g164(.a(new_n253), .b(new_n248), .c(new_n245), .d(new_n252), .out0(new_n260));
  nano23aa1n09x5               g165(.a(new_n214), .b(new_n260), .c(new_n259), .d(new_n205), .out0(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  nano23aa1n03x7               g167(.a(new_n245), .b(new_n252), .c(new_n253), .d(new_n248), .out0(new_n263));
  tech160nm_fioaoi03aa1n02p5x5 g168(.a(\a[22] ), .b(\b[21] ), .c(new_n246), .o1(new_n264));
  aoi012aa1n12x5               g169(.a(new_n264), .b(new_n241), .c(new_n263), .o1(new_n265));
  aoai13aa1n04x5               g170(.a(new_n265), .b(new_n262), .c(new_n197), .d(new_n204), .o1(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g172(.a(\b[22] ), .b(\a[23] ), .o1(new_n268));
  inv000aa1n02x5               g173(.a(new_n268), .o1(new_n269));
  inv000aa1n02x5               g174(.a(new_n265), .o1(new_n270));
  xorc02aa1n06x5               g175(.a(\a[23] ), .b(\b[22] ), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n270), .c(new_n225), .d(new_n261), .o1(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[23] ), .b(\a[24] ), .out0(new_n273));
  tech160nm_fiaoi012aa1n02p5x5 g178(.a(new_n273), .b(new_n272), .c(new_n269), .o1(new_n274));
  xorc02aa1n03x5               g179(.a(\a[24] ), .b(\b[23] ), .out0(new_n275));
  aoi112aa1n03x4               g180(.a(new_n268), .b(new_n275), .c(new_n266), .d(new_n271), .o1(new_n276));
  nor002aa1n02x5               g181(.a(new_n274), .b(new_n276), .o1(\s[24] ));
  nand23aa1n03x5               g182(.a(new_n263), .b(new_n271), .c(new_n275), .o1(new_n278));
  norp02aa1n02x5               g183(.a(new_n238), .b(new_n278), .o1(new_n279));
  inv000aa1n02x5               g184(.a(new_n279), .o1(new_n280));
  xnrc02aa1n02x5               g185(.a(\b[22] ), .b(\a[23] ), .out0(new_n281));
  nor043aa1n03x5               g186(.a(new_n260), .b(new_n281), .c(new_n273), .o1(new_n282));
  nand22aa1n12x5               g187(.a(new_n241), .b(new_n282), .o1(new_n283));
  oaoi03aa1n02x5               g188(.a(\a[24] ), .b(\b[23] ), .c(new_n269), .o1(new_n284));
  aoi013aa1n06x4               g189(.a(new_n284), .b(new_n264), .c(new_n271), .d(new_n275), .o1(new_n285));
  nand02aa1d10x5               g190(.a(new_n283), .b(new_n285), .o1(new_n286));
  inv000aa1n09x5               g191(.a(new_n286), .o1(new_n287));
  aoai13aa1n06x5               g192(.a(new_n287), .b(new_n280), .c(new_n197), .d(new_n204), .o1(new_n288));
  xorb03aa1n02x5               g193(.a(new_n288), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g194(.a(\b[24] ), .b(\a[25] ), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  xorc02aa1n12x5               g196(.a(\a[25] ), .b(\b[24] ), .out0(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n286), .c(new_n225), .d(new_n279), .o1(new_n293));
  xorc02aa1n12x5               g198(.a(\a[26] ), .b(\b[25] ), .out0(new_n294));
  inv000aa1d42x5               g199(.a(new_n294), .o1(new_n295));
  tech160nm_fiaoi012aa1n05x5   g200(.a(new_n295), .b(new_n293), .c(new_n291), .o1(new_n296));
  aoi112aa1n03x4               g201(.a(new_n290), .b(new_n294), .c(new_n288), .d(new_n292), .o1(new_n297));
  nor002aa1n02x5               g202(.a(new_n296), .b(new_n297), .o1(\s[26] ));
  and002aa1n18x5               g203(.a(new_n294), .b(new_n292), .o(new_n299));
  norb03aa1d15x5               g204(.a(new_n299), .b(new_n238), .c(new_n278), .out0(new_n300));
  aoai13aa1n06x5               g205(.a(new_n300), .b(new_n210), .c(new_n126), .d(new_n196), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[26] ), .b(\b[25] ), .c(new_n291), .carry(new_n302));
  aobi12aa1n18x5               g207(.a(new_n302), .b(new_n286), .c(new_n299), .out0(new_n303));
  xorc02aa1n12x5               g208(.a(\a[27] ), .b(\b[26] ), .out0(new_n304));
  xnbna2aa1n03x5               g209(.a(new_n304), .b(new_n301), .c(new_n303), .out0(\s[27] ));
  norp02aa1n02x5               g210(.a(\b[26] ), .b(\a[27] ), .o1(new_n306));
  inv040aa1n03x5               g211(.a(new_n306), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n299), .o1(new_n308));
  aoai13aa1n06x5               g213(.a(new_n302), .b(new_n308), .c(new_n283), .d(new_n285), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n304), .b(new_n309), .c(new_n225), .d(new_n300), .o1(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[27] ), .b(\a[28] ), .out0(new_n311));
  tech160nm_fiaoi012aa1n02p5x5 g216(.a(new_n311), .b(new_n310), .c(new_n307), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n304), .o1(new_n313));
  aoi012aa1n06x5               g218(.a(new_n313), .b(new_n301), .c(new_n303), .o1(new_n314));
  nano22aa1n03x5               g219(.a(new_n314), .b(new_n307), .c(new_n311), .out0(new_n315));
  norp02aa1n03x5               g220(.a(new_n312), .b(new_n315), .o1(\s[28] ));
  norb02aa1n09x5               g221(.a(new_n304), .b(new_n311), .out0(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n309), .c(new_n225), .d(new_n300), .o1(new_n318));
  oao003aa1n02x5               g223(.a(\a[28] ), .b(\b[27] ), .c(new_n307), .carry(new_n319));
  xnrc02aa1n02x5               g224(.a(\b[28] ), .b(\a[29] ), .out0(new_n320));
  tech160nm_fiaoi012aa1n02p5x5 g225(.a(new_n320), .b(new_n318), .c(new_n319), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n317), .o1(new_n322));
  tech160nm_fiaoi012aa1n03p5x5 g227(.a(new_n322), .b(new_n301), .c(new_n303), .o1(new_n323));
  nano22aa1n03x7               g228(.a(new_n323), .b(new_n319), .c(new_n320), .out0(new_n324));
  norp02aa1n03x5               g229(.a(new_n321), .b(new_n324), .o1(\s[29] ));
  xorb03aa1n02x5               g230(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g231(.a(new_n304), .b(new_n320), .c(new_n311), .out0(new_n327));
  aoai13aa1n03x5               g232(.a(new_n327), .b(new_n309), .c(new_n225), .d(new_n300), .o1(new_n328));
  oao003aa1n02x5               g233(.a(\a[29] ), .b(\b[28] ), .c(new_n319), .carry(new_n329));
  xnrc02aa1n02x5               g234(.a(\b[29] ), .b(\a[30] ), .out0(new_n330));
  tech160nm_fiaoi012aa1n02p5x5 g235(.a(new_n330), .b(new_n328), .c(new_n329), .o1(new_n331));
  inv000aa1d42x5               g236(.a(new_n327), .o1(new_n332));
  aoi012aa1n06x5               g237(.a(new_n332), .b(new_n301), .c(new_n303), .o1(new_n333));
  nano22aa1n03x5               g238(.a(new_n333), .b(new_n329), .c(new_n330), .out0(new_n334));
  norp02aa1n03x5               g239(.a(new_n331), .b(new_n334), .o1(\s[30] ));
  xnrc02aa1n02x5               g240(.a(\b[30] ), .b(\a[31] ), .out0(new_n336));
  nona32aa1n02x4               g241(.a(new_n304), .b(new_n330), .c(new_n320), .d(new_n311), .out0(new_n337));
  inv000aa1n02x5               g242(.a(new_n337), .o1(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n309), .c(new_n225), .d(new_n300), .o1(new_n339));
  oao003aa1n02x5               g244(.a(\a[30] ), .b(\b[29] ), .c(new_n329), .carry(new_n340));
  tech160nm_fiaoi012aa1n02p5x5 g245(.a(new_n336), .b(new_n339), .c(new_n340), .o1(new_n341));
  aoi012aa1n06x5               g246(.a(new_n337), .b(new_n301), .c(new_n303), .o1(new_n342));
  nano22aa1n03x5               g247(.a(new_n342), .b(new_n336), .c(new_n340), .out0(new_n343));
  norp02aa1n03x5               g248(.a(new_n341), .b(new_n343), .o1(\s[31] ));
  xnbna2aa1n03x5               g249(.a(new_n102), .b(new_n105), .c(new_n149), .out0(\s[3] ));
  oaoi03aa1n02x5               g250(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n346));
  xorb03aa1n02x5               g251(.a(new_n346), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g252(.a(new_n168), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g253(.a(new_n122), .b(new_n123), .c(new_n168), .o1(new_n349));
  xnrb03aa1n02x5               g254(.a(new_n349), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  orn002aa1n02x5               g255(.a(new_n113), .b(new_n114), .o(new_n351));
  aoai13aa1n02x5               g256(.a(new_n124), .b(new_n351), .c(new_n107), .d(new_n110), .o1(new_n352));
  xorb03aa1n02x5               g257(.a(new_n352), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g258(.a(new_n117), .b(new_n352), .c(new_n118), .o1(new_n354));
  xnrb03aa1n02x5               g259(.a(new_n354), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g260(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


