// Benchmark "adder" written by ABC on Wed Jul 17 15:20:09 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n291, new_n292, new_n293, new_n294, new_n295,
    new_n296, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n325, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n350, new_n351,
    new_n352, new_n353, new_n355, new_n356, new_n359, new_n360, new_n361,
    new_n362, new_n364, new_n365, new_n366, new_n368, new_n370, new_n371,
    new_n372;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  and002aa1n12x5               g001(.a(\b[0] ), .b(\a[1] ), .o(new_n97));
  oaoi03aa1n12x5               g002(.a(\a[2] ), .b(\b[1] ), .c(new_n97), .o1(new_n98));
  nand42aa1n10x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nor042aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  norb02aa1n06x4               g005(.a(new_n99), .b(new_n100), .out0(new_n101));
  nor042aa1n04x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanp02aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  norb02aa1n06x4               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  nanp03aa1d12x5               g009(.a(new_n98), .b(new_n101), .c(new_n104), .o1(new_n105));
  tech160nm_fiaoi012aa1n04x5   g010(.a(new_n100), .b(new_n102), .c(new_n99), .o1(new_n106));
  nand42aa1n10x5               g011(.a(\b[7] ), .b(\a[8] ), .o1(new_n107));
  nor002aa1n03x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nor002aa1n16x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nand42aa1d28x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nano23aa1n06x5               g015(.a(new_n109), .b(new_n108), .c(new_n110), .d(new_n107), .out0(new_n111));
  xorc02aa1n12x5               g016(.a(\a[5] ), .b(\b[4] ), .out0(new_n112));
  xorc02aa1n12x5               g017(.a(\a[6] ), .b(\b[5] ), .out0(new_n113));
  nand23aa1d12x5               g018(.a(new_n111), .b(new_n112), .c(new_n113), .o1(new_n114));
  oaih22aa1d12x5               g019(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n115));
  inv000aa1d42x5               g020(.a(new_n110), .o1(new_n116));
  nor042aa1n02x5               g021(.a(new_n115), .b(new_n116), .o1(new_n117));
  nand02aa1d28x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  oaih22aa1d12x5               g023(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n119));
  nanp03aa1n02x5               g024(.a(new_n119), .b(new_n107), .c(new_n118), .o1(new_n120));
  aboi22aa1n06x5               g025(.a(new_n120), .b(new_n117), .c(new_n115), .d(new_n107), .out0(new_n121));
  aoai13aa1n12x5               g026(.a(new_n121), .b(new_n114), .c(new_n105), .d(new_n106), .o1(new_n122));
  norp02aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  tech160nm_fixorc02aa1n03p5x5 g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  tech160nm_fixorc02aa1n03p5x5 g029(.a(\a[10] ), .b(\b[9] ), .out0(new_n125));
  aoi112aa1n02x5               g030(.a(new_n123), .b(new_n125), .c(new_n122), .d(new_n124), .o1(new_n126));
  aoai13aa1n06x5               g031(.a(new_n125), .b(new_n123), .c(new_n122), .d(new_n124), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(\s[10] ));
  oai022aa1d18x5               g033(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n129));
  aobi12aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(new_n130));
  inv000aa1d42x5               g035(.a(new_n130), .o1(new_n131));
  nor042aa1n12x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand22aa1n04x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n06x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n127), .c(new_n131), .out0(\s[11] ));
  aob012aa1n03x5               g040(.a(new_n134), .b(new_n127), .c(new_n131), .out0(new_n136));
  nor002aa1d32x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand02aa1n06x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  inv000aa1d42x5               g044(.a(new_n137), .o1(new_n140));
  aoi012aa1n02x5               g045(.a(new_n132), .b(new_n140), .c(new_n138), .o1(new_n141));
  inv040aa1n03x5               g046(.a(new_n132), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n134), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n142), .b(new_n143), .c(new_n127), .d(new_n131), .o1(new_n144));
  aoi022aa1n03x5               g049(.a(new_n144), .b(new_n139), .c(new_n136), .d(new_n141), .o1(\s[12] ));
  nand42aa1n04x5               g050(.a(new_n105), .b(new_n106), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n114), .o1(new_n147));
  oai012aa1n02x5               g052(.a(new_n107), .b(new_n109), .c(new_n108), .o1(new_n148));
  oaib12aa1n02x5               g053(.a(new_n148), .b(new_n120), .c(new_n117), .out0(new_n149));
  nano23aa1n03x7               g054(.a(new_n132), .b(new_n137), .c(new_n138), .d(new_n133), .out0(new_n150));
  and003aa1n02x5               g055(.a(new_n150), .b(new_n125), .c(new_n124), .o(new_n151));
  aoai13aa1n06x5               g056(.a(new_n151), .b(new_n149), .c(new_n147), .d(new_n146), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[9] ), .b(\a[10] ), .o1(new_n153));
  nanb03aa1n03x5               g058(.a(new_n137), .b(new_n138), .c(new_n133), .out0(new_n154));
  nano32aa1n03x7               g059(.a(new_n154), .b(new_n129), .c(new_n142), .d(new_n153), .out0(new_n155));
  aoi012aa1n06x5               g060(.a(new_n137), .b(new_n132), .c(new_n138), .o1(new_n156));
  inv040aa1n03x5               g061(.a(new_n156), .o1(new_n157));
  nor042aa1n09x5               g062(.a(new_n155), .b(new_n157), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(new_n152), .b(new_n158), .o1(new_n159));
  xnrc02aa1n12x5               g064(.a(\b[12] ), .b(\a[13] ), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  nano22aa1n02x4               g066(.a(new_n155), .b(new_n156), .c(new_n160), .out0(new_n162));
  aoi022aa1n02x5               g067(.a(new_n159), .b(new_n161), .c(new_n152), .d(new_n162), .o1(\s[13] ));
  orn002aa1n02x5               g068(.a(\a[13] ), .b(\b[12] ), .o(new_n164));
  inv000aa1d42x5               g069(.a(new_n158), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n161), .b(new_n165), .c(new_n122), .d(new_n151), .o1(new_n166));
  xnrc02aa1n03x5               g071(.a(\b[13] ), .b(\a[14] ), .out0(new_n167));
  xobna2aa1n03x5               g072(.a(new_n167), .b(new_n166), .c(new_n164), .out0(\s[14] ));
  norp02aa1n02x5               g073(.a(new_n167), .b(new_n160), .o1(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n165), .c(new_n122), .d(new_n151), .o1(new_n170));
  tech160nm_fioaoi03aa1n03p5x5 g075(.a(\a[14] ), .b(\b[13] ), .c(new_n164), .o1(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  nor002aa1d32x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nand22aa1n09x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nanb02aa1d24x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n170), .c(new_n172), .out0(\s[15] ));
  aoai13aa1n02x5               g082(.a(new_n176), .b(new_n171), .c(new_n159), .d(new_n169), .o1(new_n178));
  xorc02aa1n02x5               g083(.a(\a[16] ), .b(\b[15] ), .out0(new_n179));
  inv000aa1d42x5               g084(.a(\a[16] ), .o1(new_n180));
  inv000aa1d42x5               g085(.a(\b[15] ), .o1(new_n181));
  nanp02aa1n02x5               g086(.a(new_n181), .b(new_n180), .o1(new_n182));
  nanp02aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  aoi012aa1n02x5               g088(.a(new_n173), .b(new_n182), .c(new_n183), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n173), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n185), .b(new_n175), .c(new_n170), .d(new_n172), .o1(new_n186));
  aoi022aa1n02x7               g091(.a(new_n186), .b(new_n179), .c(new_n178), .d(new_n184), .o1(\s[16] ));
  nano22aa1n03x7               g092(.a(new_n175), .b(new_n182), .c(new_n183), .out0(new_n188));
  nona22aa1n09x5               g093(.a(new_n188), .b(new_n167), .c(new_n160), .out0(new_n189));
  nano32aa1d12x5               g094(.a(new_n189), .b(new_n150), .c(new_n125), .d(new_n124), .out0(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n149), .c(new_n147), .d(new_n146), .o1(new_n191));
  nanp03aa1n02x5               g096(.a(new_n182), .b(new_n174), .c(new_n183), .o1(new_n192));
  inv000aa1d42x5               g097(.a(\a[14] ), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\b[13] ), .o1(new_n194));
  oaih22aa1d12x5               g099(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n195));
  oai122aa1n02x7               g100(.a(new_n195), .b(\a[15] ), .c(\b[14] ), .d(new_n193), .e(new_n194), .o1(new_n196));
  oaoi03aa1n02x5               g101(.a(new_n180), .b(new_n181), .c(new_n173), .o1(new_n197));
  oai012aa1n02x7               g102(.a(new_n197), .b(new_n196), .c(new_n192), .o1(new_n198));
  oabi12aa1n09x5               g103(.a(new_n198), .b(new_n158), .c(new_n189), .out0(new_n199));
  inv000aa1n06x5               g104(.a(new_n199), .o1(new_n200));
  nand02aa1d08x5               g105(.a(new_n191), .b(new_n200), .o1(new_n201));
  xorc02aa1n12x5               g106(.a(\a[17] ), .b(\b[16] ), .out0(new_n202));
  orn002aa1n02x5               g107(.a(new_n196), .b(new_n192), .o(new_n203));
  nanb03aa1n02x5               g108(.a(new_n202), .b(new_n203), .c(new_n197), .out0(new_n204));
  aoib12aa1n02x5               g109(.a(new_n204), .b(new_n165), .c(new_n189), .out0(new_n205));
  aoi022aa1n02x5               g110(.a(new_n201), .b(new_n202), .c(new_n191), .d(new_n205), .o1(\s[17] ));
  nor002aa1d32x5               g111(.a(\b[16] ), .b(\a[17] ), .o1(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n02x5               g113(.a(new_n202), .b(new_n199), .c(new_n122), .d(new_n190), .o1(new_n209));
  nor042aa1n06x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  nand02aa1d16x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  norb02aa1n06x4               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n209), .c(new_n208), .out0(\s[18] ));
  and002aa1n02x5               g118(.a(new_n202), .b(new_n212), .o(new_n214));
  aoai13aa1n03x5               g119(.a(new_n214), .b(new_n199), .c(new_n122), .d(new_n190), .o1(new_n215));
  oaoi03aa1n02x5               g120(.a(\a[18] ), .b(\b[17] ), .c(new_n208), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  nor042aa1d18x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  nand42aa1n06x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  norb02aa1n06x4               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  xnbna2aa1n03x5               g125(.a(new_n220), .b(new_n215), .c(new_n217), .out0(\s[19] ));
  xnrc02aa1n02x5               g126(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g127(.a(new_n220), .b(new_n216), .c(new_n201), .d(new_n214), .o1(new_n223));
  nor042aa1n06x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nand02aa1d20x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  inv000aa1d42x5               g131(.a(\a[19] ), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\b[18] ), .o1(new_n228));
  aboi22aa1n03x5               g133(.a(new_n224), .b(new_n225), .c(new_n227), .d(new_n228), .out0(new_n229));
  inv040aa1n02x5               g134(.a(new_n218), .o1(new_n230));
  inv000aa1n06x5               g135(.a(new_n220), .o1(new_n231));
  aoai13aa1n02x5               g136(.a(new_n230), .b(new_n231), .c(new_n215), .d(new_n217), .o1(new_n232));
  aoi022aa1n03x5               g137(.a(new_n232), .b(new_n226), .c(new_n223), .d(new_n229), .o1(\s[20] ));
  nano32aa1n03x7               g138(.a(new_n231), .b(new_n202), .c(new_n226), .d(new_n212), .out0(new_n234));
  aoai13aa1n02x5               g139(.a(new_n234), .b(new_n199), .c(new_n122), .d(new_n190), .o1(new_n235));
  nanb03aa1n12x5               g140(.a(new_n224), .b(new_n225), .c(new_n219), .out0(new_n236));
  oai112aa1n06x5               g141(.a(new_n230), .b(new_n211), .c(new_n210), .d(new_n207), .o1(new_n237));
  aoi012aa1n12x5               g142(.a(new_n224), .b(new_n218), .c(new_n225), .o1(new_n238));
  oaih12aa1n12x5               g143(.a(new_n238), .b(new_n237), .c(new_n236), .o1(new_n239));
  nor002aa1d32x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  nand42aa1n04x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  norb02aa1n09x5               g146(.a(new_n241), .b(new_n240), .out0(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n239), .c(new_n201), .d(new_n234), .o1(new_n243));
  nano22aa1n03x5               g148(.a(new_n224), .b(new_n219), .c(new_n225), .out0(new_n244));
  tech160nm_fioai012aa1n03p5x5 g149(.a(new_n211), .b(\b[18] ), .c(\a[19] ), .o1(new_n245));
  oab012aa1n06x5               g150(.a(new_n245), .b(new_n207), .c(new_n210), .out0(new_n246));
  inv020aa1n03x5               g151(.a(new_n238), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n247), .b(new_n242), .c(new_n246), .d(new_n244), .o1(new_n248));
  aobi12aa1n02x7               g153(.a(new_n243), .b(new_n248), .c(new_n235), .out0(\s[21] ));
  nor042aa1n03x5               g154(.a(\b[21] ), .b(\a[22] ), .o1(new_n250));
  nanp02aa1n04x5               g155(.a(\b[21] ), .b(\a[22] ), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n251), .b(new_n250), .out0(new_n252));
  aoib12aa1n02x5               g157(.a(new_n240), .b(new_n251), .c(new_n250), .out0(new_n253));
  inv000aa1d42x5               g158(.a(new_n239), .o1(new_n254));
  inv040aa1n08x5               g159(.a(new_n240), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n242), .o1(new_n256));
  aoai13aa1n03x5               g161(.a(new_n255), .b(new_n256), .c(new_n235), .d(new_n254), .o1(new_n257));
  aoi022aa1n03x5               g162(.a(new_n257), .b(new_n252), .c(new_n243), .d(new_n253), .o1(\s[22] ));
  inv000aa1n02x5               g163(.a(new_n234), .o1(new_n259));
  nano22aa1n03x7               g164(.a(new_n259), .b(new_n242), .c(new_n252), .out0(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n199), .c(new_n122), .d(new_n190), .o1(new_n261));
  nano23aa1d15x5               g166(.a(new_n240), .b(new_n250), .c(new_n251), .d(new_n241), .out0(new_n262));
  oaoi03aa1n12x5               g167(.a(\a[22] ), .b(\b[21] ), .c(new_n255), .o1(new_n263));
  aoi012aa1n12x5               g168(.a(new_n263), .b(new_n239), .c(new_n262), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  xorc02aa1n12x5               g170(.a(\a[23] ), .b(\b[22] ), .out0(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n265), .c(new_n201), .d(new_n260), .o1(new_n267));
  aoi112aa1n02x5               g172(.a(new_n266), .b(new_n263), .c(new_n239), .d(new_n262), .o1(new_n268));
  aobi12aa1n02x7               g173(.a(new_n267), .b(new_n268), .c(new_n261), .out0(\s[23] ));
  tech160nm_fixorc02aa1n02p5x5 g174(.a(\a[24] ), .b(\b[23] ), .out0(new_n270));
  nor042aa1n06x5               g175(.a(\b[22] ), .b(\a[23] ), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n270), .b(new_n271), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n271), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n266), .o1(new_n274));
  aoai13aa1n03x5               g179(.a(new_n273), .b(new_n274), .c(new_n261), .d(new_n264), .o1(new_n275));
  aoi022aa1n02x7               g180(.a(new_n275), .b(new_n270), .c(new_n267), .d(new_n272), .o1(\s[24] ));
  and002aa1n12x5               g181(.a(new_n270), .b(new_n266), .o(new_n277));
  nano22aa1n03x7               g182(.a(new_n259), .b(new_n277), .c(new_n262), .out0(new_n278));
  aoai13aa1n02x5               g183(.a(new_n278), .b(new_n199), .c(new_n122), .d(new_n190), .o1(new_n279));
  aoai13aa1n04x5               g184(.a(new_n262), .b(new_n247), .c(new_n246), .d(new_n244), .o1(new_n280));
  inv000aa1n02x5               g185(.a(new_n263), .o1(new_n281));
  inv000aa1n06x5               g186(.a(new_n277), .o1(new_n282));
  oao003aa1n03x5               g187(.a(\a[24] ), .b(\b[23] ), .c(new_n273), .carry(new_n283));
  aoai13aa1n12x5               g188(.a(new_n283), .b(new_n282), .c(new_n280), .d(new_n281), .o1(new_n284));
  xorc02aa1n12x5               g189(.a(\a[25] ), .b(\b[24] ), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n285), .b(new_n284), .c(new_n201), .d(new_n278), .o1(new_n286));
  aoai13aa1n06x5               g191(.a(new_n277), .b(new_n263), .c(new_n239), .d(new_n262), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n285), .o1(new_n288));
  and003aa1n02x5               g193(.a(new_n287), .b(new_n288), .c(new_n283), .o(new_n289));
  aobi12aa1n03x7               g194(.a(new_n286), .b(new_n289), .c(new_n279), .out0(\s[25] ));
  tech160nm_fixorc02aa1n02p5x5 g195(.a(\a[26] ), .b(\b[25] ), .out0(new_n291));
  nor042aa1n03x5               g196(.a(\b[24] ), .b(\a[25] ), .o1(new_n292));
  norp02aa1n02x5               g197(.a(new_n291), .b(new_n292), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n284), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n292), .o1(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n288), .c(new_n279), .d(new_n294), .o1(new_n296));
  aoi022aa1n03x5               g201(.a(new_n296), .b(new_n291), .c(new_n286), .d(new_n293), .o1(\s[26] ));
  and002aa1n12x5               g202(.a(new_n291), .b(new_n285), .o(new_n298));
  nano32aa1n03x7               g203(.a(new_n259), .b(new_n298), .c(new_n262), .d(new_n277), .out0(new_n299));
  aoai13aa1n06x5               g204(.a(new_n299), .b(new_n199), .c(new_n122), .d(new_n190), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n298), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[26] ), .b(\b[25] ), .c(new_n295), .carry(new_n302));
  aoai13aa1n04x5               g207(.a(new_n302), .b(new_n301), .c(new_n287), .d(new_n283), .o1(new_n303));
  xorc02aa1n12x5               g208(.a(\a[27] ), .b(\b[26] ), .out0(new_n304));
  aoai13aa1n06x5               g209(.a(new_n304), .b(new_n303), .c(new_n201), .d(new_n299), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n302), .o1(new_n306));
  aoi112aa1n02x5               g211(.a(new_n304), .b(new_n306), .c(new_n284), .d(new_n298), .o1(new_n307));
  aobi12aa1n02x7               g212(.a(new_n305), .b(new_n307), .c(new_n300), .out0(\s[27] ));
  tech160nm_fixorc02aa1n02p5x5 g213(.a(\a[28] ), .b(\b[27] ), .out0(new_n309));
  norp02aa1n02x5               g214(.a(\b[26] ), .b(\a[27] ), .o1(new_n310));
  norp02aa1n02x5               g215(.a(new_n309), .b(new_n310), .o1(new_n311));
  tech160nm_fiaoi012aa1n05x5   g216(.a(new_n306), .b(new_n284), .c(new_n298), .o1(new_n312));
  inv000aa1n03x5               g217(.a(new_n310), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n304), .o1(new_n314));
  aoai13aa1n02x7               g219(.a(new_n313), .b(new_n314), .c(new_n312), .d(new_n300), .o1(new_n315));
  aoi022aa1n02x7               g220(.a(new_n315), .b(new_n309), .c(new_n305), .d(new_n311), .o1(\s[28] ));
  and002aa1n02x5               g221(.a(new_n309), .b(new_n304), .o(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n303), .c(new_n201), .d(new_n299), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n317), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[28] ), .b(\b[27] ), .c(new_n313), .carry(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n319), .c(new_n312), .d(new_n300), .o1(new_n321));
  xorc02aa1n02x5               g226(.a(\a[29] ), .b(\b[28] ), .out0(new_n322));
  norb02aa1n02x5               g227(.a(new_n320), .b(new_n322), .out0(new_n323));
  aoi022aa1n03x5               g228(.a(new_n321), .b(new_n322), .c(new_n318), .d(new_n323), .o1(\s[29] ));
  inv000aa1d42x5               g229(.a(\a[2] ), .o1(new_n325));
  xorb03aa1n02x5               g230(.a(new_n97), .b(\b[1] ), .c(new_n325), .out0(\s[2] ));
  nano22aa1n03x7               g231(.a(new_n314), .b(new_n309), .c(new_n322), .out0(new_n327));
  aoai13aa1n03x5               g232(.a(new_n327), .b(new_n303), .c(new_n201), .d(new_n299), .o1(new_n328));
  inv000aa1d42x5               g233(.a(new_n327), .o1(new_n329));
  inv000aa1d42x5               g234(.a(\b[28] ), .o1(new_n330));
  inv000aa1d42x5               g235(.a(\a[29] ), .o1(new_n331));
  oaib12aa1n02x5               g236(.a(new_n320), .b(\b[28] ), .c(new_n331), .out0(new_n332));
  oaib12aa1n02x5               g237(.a(new_n332), .b(new_n330), .c(\a[29] ), .out0(new_n333));
  aoai13aa1n03x5               g238(.a(new_n333), .b(new_n329), .c(new_n312), .d(new_n300), .o1(new_n334));
  xorc02aa1n02x5               g239(.a(\a[30] ), .b(\b[29] ), .out0(new_n335));
  oaoi13aa1n02x5               g240(.a(new_n335), .b(new_n332), .c(new_n331), .d(new_n330), .o1(new_n336));
  aoi022aa1n03x5               g241(.a(new_n334), .b(new_n335), .c(new_n328), .d(new_n336), .o1(\s[30] ));
  nano32aa1n06x5               g242(.a(new_n314), .b(new_n335), .c(new_n309), .d(new_n322), .out0(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n303), .c(new_n201), .d(new_n299), .o1(new_n339));
  aoi022aa1n02x5               g244(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n340));
  norb02aa1n02x5               g245(.a(\b[30] ), .b(\a[31] ), .out0(new_n341));
  obai22aa1n02x7               g246(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n342));
  aoi112aa1n02x5               g247(.a(new_n342), .b(new_n341), .c(new_n332), .d(new_n340), .o1(new_n343));
  xorc02aa1n02x5               g248(.a(\a[31] ), .b(\b[30] ), .out0(new_n344));
  inv000aa1d42x5               g249(.a(new_n338), .o1(new_n345));
  norp02aa1n02x5               g250(.a(\b[29] ), .b(\a[30] ), .o1(new_n346));
  aoi012aa1n02x5               g251(.a(new_n346), .b(new_n332), .c(new_n340), .o1(new_n347));
  aoai13aa1n03x5               g252(.a(new_n347), .b(new_n345), .c(new_n312), .d(new_n300), .o1(new_n348));
  aoi022aa1n03x5               g253(.a(new_n348), .b(new_n344), .c(new_n339), .d(new_n343), .o1(\s[31] ));
  inv000aa1d42x5               g254(.a(\b[1] ), .o1(new_n350));
  aoi022aa1n02x5               g255(.a(new_n350), .b(new_n325), .c(\a[1] ), .d(\b[0] ), .o1(new_n351));
  oaib12aa1n02x5               g256(.a(new_n351), .b(new_n350), .c(\a[2] ), .out0(new_n352));
  aboi22aa1n03x5               g257(.a(new_n102), .b(new_n103), .c(new_n325), .d(new_n350), .out0(new_n353));
  aoi022aa1n02x5               g258(.a(new_n352), .b(new_n353), .c(new_n98), .d(new_n104), .o1(\s[3] ));
  aoai13aa1n02x5               g259(.a(new_n101), .b(new_n102), .c(new_n98), .d(new_n103), .o1(new_n355));
  aoi112aa1n02x5               g260(.a(new_n102), .b(new_n101), .c(new_n98), .d(new_n103), .o1(new_n356));
  norb02aa1n02x5               g261(.a(new_n355), .b(new_n356), .out0(\s[4] ));
  xnbna2aa1n03x5               g262(.a(new_n112), .b(new_n105), .c(new_n106), .out0(\s[5] ));
  nanp02aa1n02x5               g263(.a(new_n146), .b(new_n112), .o1(new_n359));
  oaoi13aa1n02x5               g264(.a(new_n113), .b(new_n359), .c(\a[5] ), .d(\b[4] ), .o1(new_n360));
  inv000aa1d42x5               g265(.a(new_n118), .o1(new_n361));
  nona22aa1n02x4               g266(.a(new_n359), .b(new_n119), .c(new_n361), .out0(new_n362));
  nanb02aa1n02x5               g267(.a(new_n360), .b(new_n362), .out0(\s[6] ));
  inv000aa1d42x5               g268(.a(new_n109), .o1(new_n364));
  aoi022aa1n02x5               g269(.a(new_n362), .b(new_n118), .c(new_n364), .d(new_n110), .o1(new_n365));
  nona32aa1n02x4               g270(.a(new_n362), .b(new_n361), .c(new_n116), .d(new_n109), .out0(new_n366));
  norb02aa1n02x5               g271(.a(new_n366), .b(new_n365), .out0(\s[7] ));
  norb02aa1n02x5               g272(.a(new_n107), .b(new_n108), .out0(new_n368));
  xnbna2aa1n03x5               g273(.a(new_n368), .b(new_n366), .c(new_n364), .out0(\s[8] ));
  nanp02aa1n02x5               g274(.a(new_n147), .b(new_n146), .o1(new_n370));
  nanb02aa1n02x5               g275(.a(new_n124), .b(new_n148), .out0(new_n371));
  aoib12aa1n02x5               g276(.a(new_n371), .b(new_n117), .c(new_n120), .out0(new_n372));
  aoi022aa1n02x5               g277(.a(new_n122), .b(new_n124), .c(new_n370), .d(new_n372), .o1(\s[9] ));
endmodule


