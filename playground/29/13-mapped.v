// Benchmark "adder" written by ABC on Thu Jul 18 02:54:47 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n322, new_n323, new_n325,
    new_n327, new_n329, new_n331, new_n332, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[9] ), .o1(new_n97));
  inv030aa1d32x5               g002(.a(\b[8] ), .o1(new_n98));
  nand02aa1n03x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  orn002aa1n03x5               g004(.a(\a[2] ), .b(\b[1] ), .o(new_n100));
  nanp02aa1n04x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nand42aa1n10x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  aob012aa1n06x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .out0(new_n103));
  nor042aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n09x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb02aa1n03x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nor002aa1d32x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanp02aa1n04x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  norb02aa1n03x5               g013(.a(new_n108), .b(new_n107), .out0(new_n109));
  nand23aa1n06x5               g014(.a(new_n103), .b(new_n106), .c(new_n109), .o1(new_n110));
  oaih12aa1n02x5               g015(.a(new_n105), .b(new_n107), .c(new_n104), .o1(new_n111));
  nor002aa1n03x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nand02aa1n12x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nor042aa1n09x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nand02aa1n04x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nano23aa1n03x7               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  nor002aa1d24x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand42aa1n16x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nor042aa1n03x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nand02aa1n08x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nano23aa1n09x5               g025(.a(new_n117), .b(new_n119), .c(new_n120), .d(new_n118), .out0(new_n121));
  nand22aa1n03x5               g026(.a(new_n121), .b(new_n116), .o1(new_n122));
  ao0012aa1n03x7               g027(.a(new_n112), .b(new_n114), .c(new_n113), .o(new_n123));
  inv040aa1n04x5               g028(.a(new_n117), .o1(new_n124));
  oai112aa1n06x5               g029(.a(new_n124), .b(new_n118), .c(\b[6] ), .d(\a[7] ), .o1(new_n125));
  aoi022aa1n09x5               g030(.a(new_n121), .b(new_n123), .c(new_n118), .d(new_n125), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n126), .b(new_n122), .c(new_n110), .d(new_n111), .o1(new_n127));
  tech160nm_fixorc02aa1n03p5x5 g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  nanp02aa1n02x5               g033(.a(new_n127), .b(new_n128), .o1(new_n129));
  tech160nm_fixorc02aa1n03p5x5 g034(.a(\a[10] ), .b(\b[9] ), .out0(new_n130));
  and002aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o(new_n131));
  oai022aa1n02x5               g036(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n132));
  nona22aa1n02x4               g037(.a(new_n129), .b(new_n131), .c(new_n132), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n130), .c(new_n99), .d(new_n129), .o1(\s[10] ));
  and002aa1n02x5               g039(.a(new_n130), .b(new_n128), .o(new_n135));
  oaoi03aa1n12x5               g040(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n136));
  tech160nm_fiaoi012aa1n05x5   g041(.a(new_n136), .b(new_n127), .c(new_n135), .o1(new_n137));
  nor002aa1n16x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n138), .o1(new_n139));
  nand02aa1n06x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n137), .b(new_n140), .c(new_n139), .out0(\s[11] ));
  nanb03aa1n03x5               g046(.a(new_n137), .b(new_n140), .c(new_n139), .out0(new_n142));
  nor002aa1n03x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand02aa1d08x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  nona23aa1n03x5               g050(.a(new_n142), .b(new_n144), .c(new_n143), .d(new_n138), .out0(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n145), .c(new_n142), .d(new_n139), .o1(\s[12] ));
  nano23aa1n06x5               g052(.a(new_n138), .b(new_n143), .c(new_n144), .d(new_n140), .out0(new_n148));
  and003aa1n02x5               g053(.a(new_n148), .b(new_n130), .c(new_n128), .o(new_n149));
  tech160nm_fioai012aa1n03p5x5 g054(.a(new_n144), .b(new_n143), .c(new_n138), .o1(new_n150));
  nand02aa1n02x5               g055(.a(new_n148), .b(new_n136), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n151), .b(new_n150), .o1(new_n152));
  tech160nm_fiao0012aa1n02p5x5 g057(.a(new_n152), .b(new_n127), .c(new_n149), .o(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1d18x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  nand42aa1n04x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nanp03aa1n03x5               g062(.a(new_n153), .b(new_n156), .c(new_n157), .o1(new_n158));
  nor042aa1n04x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n09x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  nona23aa1n02x4               g066(.a(new_n158), .b(new_n160), .c(new_n159), .d(new_n155), .out0(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n161), .c(new_n156), .d(new_n158), .o1(\s[14] ));
  nano23aa1n09x5               g068(.a(new_n155), .b(new_n159), .c(new_n160), .d(new_n157), .out0(new_n164));
  aoai13aa1n02x7               g069(.a(new_n164), .b(new_n152), .c(new_n127), .d(new_n149), .o1(new_n165));
  oai012aa1n06x5               g070(.a(new_n160), .b(new_n159), .c(new_n155), .o1(new_n166));
  nor042aa1n12x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nand02aa1n03x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  xobna2aa1n03x5               g074(.a(new_n169), .b(new_n165), .c(new_n166), .out0(\s[15] ));
  inv000aa1d42x5               g075(.a(new_n167), .o1(new_n171));
  tech160nm_fiao0012aa1n03p5x5 g076(.a(new_n169), .b(new_n165), .c(new_n166), .o(new_n172));
  nor042aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nand22aa1n04x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  norb03aa1n02x5               g080(.a(new_n174), .b(new_n167), .c(new_n173), .out0(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n169), .c(new_n165), .d(new_n166), .o1(new_n177));
  aoai13aa1n03x5               g082(.a(new_n177), .b(new_n175), .c(new_n172), .d(new_n171), .o1(\s[16] ));
  nano23aa1n06x5               g083(.a(new_n167), .b(new_n173), .c(new_n174), .d(new_n168), .out0(new_n179));
  nand02aa1n02x5               g084(.a(new_n179), .b(new_n164), .o1(new_n180));
  nano32aa1n06x5               g085(.a(new_n180), .b(new_n148), .c(new_n130), .d(new_n128), .out0(new_n181));
  nanp02aa1n02x5               g086(.a(new_n127), .b(new_n181), .o1(new_n182));
  oaoi03aa1n02x5               g087(.a(\a[16] ), .b(\b[15] ), .c(new_n171), .o1(new_n183));
  aoib12aa1n02x7               g088(.a(new_n183), .b(new_n179), .c(new_n166), .out0(new_n184));
  aoai13aa1n09x5               g089(.a(new_n184), .b(new_n180), .c(new_n151), .d(new_n150), .o1(new_n185));
  nanb02aa1n02x5               g090(.a(new_n185), .b(new_n182), .out0(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1d18x5               g092(.a(\b[16] ), .b(\a[17] ), .o1(new_n188));
  inv040aa1n08x5               g093(.a(new_n188), .o1(new_n189));
  tech160nm_fixorc02aa1n04x5   g094(.a(\a[17] ), .b(\b[16] ), .out0(new_n190));
  aoai13aa1n02x5               g095(.a(new_n190), .b(new_n185), .c(new_n127), .d(new_n181), .o1(new_n191));
  xorc02aa1n12x5               g096(.a(\a[18] ), .b(\b[17] ), .out0(new_n192));
  inv000aa1d42x5               g097(.a(\a[18] ), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\b[17] ), .o1(new_n194));
  aoi012aa1n02x5               g099(.a(new_n188), .b(new_n193), .c(new_n194), .o1(new_n195));
  oai112aa1n02x5               g100(.a(new_n191), .b(new_n195), .c(new_n194), .d(new_n193), .o1(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n192), .c(new_n189), .d(new_n191), .o1(\s[18] ));
  and002aa1n02x5               g102(.a(new_n192), .b(new_n190), .o(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n185), .c(new_n127), .d(new_n181), .o1(new_n199));
  oaoi03aa1n12x5               g104(.a(\a[18] ), .b(\b[17] ), .c(new_n189), .o1(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  nor042aa1n06x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nand42aa1n04x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n199), .c(new_n201), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n02x5               g112(.a(new_n202), .o1(new_n208));
  aob012aa1n02x5               g113(.a(new_n205), .b(new_n199), .c(new_n201), .out0(new_n209));
  nor042aa1n04x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nand42aa1n06x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  norb03aa1n02x5               g117(.a(new_n211), .b(new_n202), .c(new_n210), .out0(new_n213));
  aoai13aa1n02x5               g118(.a(new_n213), .b(new_n204), .c(new_n199), .d(new_n201), .o1(new_n214));
  aoai13aa1n03x5               g119(.a(new_n214), .b(new_n212), .c(new_n209), .d(new_n208), .o1(\s[20] ));
  nano23aa1n06x5               g120(.a(new_n202), .b(new_n210), .c(new_n211), .d(new_n203), .out0(new_n216));
  nand23aa1n06x5               g121(.a(new_n216), .b(new_n190), .c(new_n192), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n04x5               g123(.a(new_n218), .b(new_n185), .c(new_n127), .d(new_n181), .o1(new_n219));
  oaoi03aa1n02x5               g124(.a(\a[20] ), .b(\b[19] ), .c(new_n208), .o1(new_n220));
  aoi012aa1n02x5               g125(.a(new_n220), .b(new_n216), .c(new_n200), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xnbna2aa1n03x5               g127(.a(new_n222), .b(new_n219), .c(new_n221), .out0(\s[21] ));
  nor042aa1n06x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  aob012aa1n03x5               g130(.a(new_n222), .b(new_n219), .c(new_n221), .out0(new_n226));
  xorc02aa1n02x5               g131(.a(\a[22] ), .b(\b[21] ), .out0(new_n227));
  nand42aa1n02x5               g132(.a(\b[21] ), .b(\a[22] ), .o1(new_n228));
  oai022aa1n02x5               g133(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n229));
  norb02aa1n02x5               g134(.a(new_n228), .b(new_n229), .out0(new_n230));
  nanp02aa1n02x5               g135(.a(new_n226), .b(new_n230), .o1(new_n231));
  aoai13aa1n02x5               g136(.a(new_n231), .b(new_n227), .c(new_n225), .d(new_n226), .o1(\s[22] ));
  nanp02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  nor042aa1n02x5               g138(.a(\b[21] ), .b(\a[22] ), .o1(new_n234));
  nano23aa1n03x7               g139(.a(new_n224), .b(new_n234), .c(new_n228), .d(new_n233), .out0(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n217), .out0(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n185), .c(new_n127), .d(new_n181), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n235), .b(new_n220), .c(new_n216), .d(new_n200), .o1(new_n238));
  oai012aa1n02x5               g143(.a(new_n228), .b(new_n234), .c(new_n224), .o1(new_n239));
  nanp02aa1n02x5               g144(.a(new_n238), .b(new_n239), .o1(new_n240));
  inv000aa1n02x5               g145(.a(new_n240), .o1(new_n241));
  xnrc02aa1n12x5               g146(.a(\b[22] ), .b(\a[23] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  xnbna2aa1n03x5               g148(.a(new_n243), .b(new_n237), .c(new_n241), .out0(\s[23] ));
  norp02aa1n02x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  aob012aa1n02x5               g151(.a(new_n243), .b(new_n237), .c(new_n241), .out0(new_n247));
  tech160nm_fixorc02aa1n04x5   g152(.a(\a[24] ), .b(\b[23] ), .out0(new_n248));
  nanp02aa1n02x5               g153(.a(\b[23] ), .b(\a[24] ), .o1(new_n249));
  oai022aa1n02x5               g154(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n249), .b(new_n250), .out0(new_n251));
  aoai13aa1n02x5               g156(.a(new_n251), .b(new_n242), .c(new_n237), .d(new_n241), .o1(new_n252));
  aoai13aa1n03x5               g157(.a(new_n252), .b(new_n248), .c(new_n247), .d(new_n246), .o1(\s[24] ));
  nano32aa1n02x4               g158(.a(new_n217), .b(new_n248), .c(new_n235), .d(new_n243), .out0(new_n254));
  aoai13aa1n02x7               g159(.a(new_n254), .b(new_n185), .c(new_n127), .d(new_n181), .o1(new_n255));
  norb02aa1n03x5               g160(.a(new_n248), .b(new_n242), .out0(new_n256));
  inv000aa1n02x5               g161(.a(new_n256), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(new_n250), .b(new_n249), .o1(new_n258));
  aoai13aa1n12x5               g163(.a(new_n258), .b(new_n257), .c(new_n238), .d(new_n239), .o1(new_n259));
  inv000aa1n02x5               g164(.a(new_n259), .o1(new_n260));
  xorc02aa1n12x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  xnbna2aa1n03x5               g166(.a(new_n261), .b(new_n255), .c(new_n260), .out0(\s[25] ));
  norp02aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  aob012aa1n03x5               g169(.a(new_n261), .b(new_n255), .c(new_n260), .out0(new_n265));
  tech160nm_fixorc02aa1n03p5x5 g170(.a(\a[26] ), .b(\b[25] ), .out0(new_n266));
  nanp02aa1n02x5               g171(.a(\b[25] ), .b(\a[26] ), .o1(new_n267));
  oai022aa1n02x5               g172(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n268));
  norb02aa1n02x5               g173(.a(new_n267), .b(new_n268), .out0(new_n269));
  nanp02aa1n03x5               g174(.a(new_n265), .b(new_n269), .o1(new_n270));
  aoai13aa1n02x5               g175(.a(new_n270), .b(new_n266), .c(new_n264), .d(new_n265), .o1(\s[26] ));
  and002aa1n02x7               g176(.a(new_n266), .b(new_n261), .o(new_n272));
  nano32aa1n03x7               g177(.a(new_n217), .b(new_n272), .c(new_n235), .d(new_n256), .out0(new_n273));
  aoai13aa1n12x5               g178(.a(new_n273), .b(new_n185), .c(new_n127), .d(new_n181), .o1(new_n274));
  aoi022aa1n12x5               g179(.a(new_n259), .b(new_n272), .c(new_n267), .d(new_n268), .o1(new_n275));
  xorc02aa1n12x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnbna2aa1n03x5               g181(.a(new_n276), .b(new_n275), .c(new_n274), .out0(\s[27] ));
  inv000aa1d42x5               g182(.a(\a[27] ), .o1(new_n278));
  nanb02aa1n02x5               g183(.a(\b[26] ), .b(new_n278), .out0(new_n279));
  nand22aa1n03x5               g184(.a(new_n259), .b(new_n272), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(new_n268), .b(new_n267), .o1(new_n281));
  nand23aa1n06x5               g186(.a(new_n280), .b(new_n274), .c(new_n281), .o1(new_n282));
  nanp02aa1n03x5               g187(.a(new_n282), .b(new_n276), .o1(new_n283));
  xorc02aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .out0(new_n284));
  inv000aa1d42x5               g189(.a(new_n276), .o1(new_n285));
  oai022aa1n02x5               g190(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(\a[28] ), .c(\b[27] ), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n285), .c(new_n275), .d(new_n274), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n284), .c(new_n283), .d(new_n279), .o1(\s[28] ));
  inv000aa1d42x5               g194(.a(\a[28] ), .o1(new_n290));
  xroi22aa1d04x5               g195(.a(new_n278), .b(\b[26] ), .c(new_n290), .d(\b[27] ), .out0(new_n291));
  inv000aa1d42x5               g196(.a(new_n291), .o1(new_n292));
  oaoi03aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n279), .o1(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  norp02aa1n02x5               g199(.a(new_n293), .b(new_n294), .o1(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n292), .c(new_n275), .d(new_n274), .o1(new_n296));
  aoai13aa1n03x5               g201(.a(new_n294), .b(new_n293), .c(new_n282), .d(new_n291), .o1(new_n297));
  nanp02aa1n03x5               g202(.a(new_n297), .b(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g203(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n09x5               g204(.a(new_n294), .b(new_n276), .c(new_n284), .out0(new_n300));
  aob012aa1n02x5               g205(.a(new_n293), .b(\b[28] ), .c(\a[29] ), .out0(new_n301));
  oai012aa1n02x5               g206(.a(new_n301), .b(\b[28] ), .c(\a[29] ), .o1(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[29] ), .b(\a[30] ), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n302), .c(new_n282), .d(new_n300), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n300), .o1(new_n305));
  norp02aa1n02x5               g210(.a(new_n302), .b(new_n303), .o1(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n305), .c(new_n275), .d(new_n274), .o1(new_n307));
  nanp02aa1n03x5               g212(.a(new_n304), .b(new_n307), .o1(\s[30] ));
  nano23aa1n02x4               g213(.a(new_n303), .b(new_n294), .c(new_n284), .d(new_n276), .out0(new_n309));
  nanp02aa1n03x5               g214(.a(new_n282), .b(new_n309), .o1(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  inv000aa1d42x5               g216(.a(new_n311), .o1(new_n312));
  inv000aa1n02x5               g217(.a(new_n309), .o1(new_n313));
  inv000aa1d42x5               g218(.a(\a[30] ), .o1(new_n314));
  oaib12aa1n02x5               g219(.a(new_n312), .b(\b[29] ), .c(new_n314), .out0(new_n315));
  aoib12aa1n02x5               g220(.a(new_n315), .b(new_n302), .c(new_n303), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n313), .c(new_n275), .d(new_n274), .o1(new_n317));
  inv000aa1d42x5               g222(.a(\b[29] ), .o1(new_n318));
  oaoi03aa1n02x5               g223(.a(new_n314), .b(new_n318), .c(new_n302), .o1(new_n319));
  aoai13aa1n03x5               g224(.a(new_n317), .b(new_n312), .c(new_n310), .d(new_n319), .o1(\s[31] ));
  xorb03aa1n02x5               g225(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  inv000aa1d42x5               g226(.a(new_n107), .o1(new_n322));
  nanp02aa1n02x5               g227(.a(new_n103), .b(new_n109), .o1(new_n323));
  xnbna2aa1n03x5               g228(.a(new_n106), .b(new_n323), .c(new_n322), .out0(\s[4] ));
  nanp02aa1n02x5               g229(.a(new_n110), .b(new_n111), .o1(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g231(.a(new_n114), .b(new_n325), .c(new_n115), .o1(new_n327));
  xnrb03aa1n02x5               g232(.a(new_n327), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g233(.a(new_n123), .b(new_n325), .c(new_n116), .o(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  norb02aa1n02x5               g235(.a(new_n120), .b(new_n119), .out0(new_n331));
  nanb02aa1n02x5               g236(.a(new_n117), .b(new_n118), .out0(new_n332));
  aoai13aa1n02x5               g237(.a(new_n332), .b(new_n119), .c(new_n329), .d(new_n120), .o1(new_n333));
  aoai13aa1n02x5               g238(.a(new_n333), .b(new_n125), .c(new_n331), .d(new_n329), .o1(\s[8] ));
  xorb03aa1n02x5               g239(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


