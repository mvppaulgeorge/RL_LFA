// Benchmark "adder" written by ABC on Wed Jul 17 15:04:40 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n255, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n300, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n341, new_n342,
    new_n343, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n353, new_n354, new_n356, new_n357, new_n358, new_n360,
    new_n361, new_n363, new_n364, new_n365, new_n368, new_n370;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand02aa1n03x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  tech160nm_fiaoi012aa1n05x5   g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  nor022aa1n04x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand42aa1n03x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor002aa1d32x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n09x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  oai012aa1n02x5               g010(.a(new_n102), .b(new_n103), .c(new_n101), .o1(new_n106));
  oai012aa1n09x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  nor002aa1n06x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nor002aa1n08x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nand02aa1d10x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nona23aa1n03x5               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xnrc02aa1n12x5               g017(.a(\b[7] ), .b(\a[8] ), .out0(new_n113));
  nor042aa1d18x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1n04x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanb02aa1n02x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  nor043aa1n04x5               g021(.a(new_n112), .b(new_n113), .c(new_n116), .o1(new_n117));
  nanp02aa1n06x5               g022(.a(new_n107), .b(new_n117), .o1(new_n118));
  norb03aa1n03x5               g023(.a(new_n111), .b(new_n108), .c(new_n110), .out0(new_n119));
  nanb03aa1d18x5               g024(.a(new_n114), .b(new_n115), .c(new_n111), .out0(new_n120));
  nor043aa1n03x5               g025(.a(new_n120), .b(new_n119), .c(new_n113), .o1(new_n121));
  inv000aa1n02x5               g026(.a(new_n114), .o1(new_n122));
  oao003aa1n02x5               g027(.a(\a[8] ), .b(\b[7] ), .c(new_n122), .carry(new_n123));
  norb02aa1n06x5               g028(.a(new_n123), .b(new_n121), .out0(new_n124));
  nor042aa1n04x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  nand42aa1n03x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  aobi12aa1n02x5               g032(.a(new_n127), .b(new_n118), .c(new_n124), .out0(new_n128));
  nand02aa1d10x5               g033(.a(new_n118), .b(new_n124), .o1(new_n129));
  aoi012aa1n02x5               g034(.a(new_n125), .b(new_n129), .c(new_n126), .o1(new_n130));
  nor042aa1n04x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand42aa1n06x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  norb03aa1n03x5               g038(.a(new_n132), .b(new_n125), .c(new_n131), .out0(new_n134));
  inv020aa1n04x5               g039(.a(new_n134), .o1(new_n135));
  oai022aa1n02x5               g040(.a(new_n130), .b(new_n133), .c(new_n135), .d(new_n128), .o1(\s[10] ));
  oai013aa1n03x5               g041(.a(new_n123), .b(new_n119), .c(new_n120), .d(new_n113), .o1(new_n137));
  nano23aa1n06x5               g042(.a(new_n125), .b(new_n131), .c(new_n132), .d(new_n126), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n138), .b(new_n137), .c(new_n107), .d(new_n117), .o1(new_n139));
  oai012aa1n02x5               g044(.a(new_n132), .b(new_n131), .c(new_n125), .o1(new_n140));
  nor042aa1n09x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  tech160nm_finand02aa1n03p5x5 g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norb02aa1n06x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n139), .c(new_n140), .out0(\s[11] ));
  inv000aa1d42x5               g049(.a(new_n141), .o1(new_n145));
  aob012aa1n03x5               g050(.a(new_n143), .b(new_n139), .c(new_n140), .out0(new_n146));
  nor002aa1n06x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nand42aa1n04x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  norb02aa1n03x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  nona23aa1n02x4               g054(.a(new_n146), .b(new_n148), .c(new_n147), .d(new_n141), .out0(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n149), .c(new_n146), .d(new_n145), .o1(\s[12] ));
  and003aa1n02x5               g056(.a(new_n138), .b(new_n149), .c(new_n143), .o(new_n152));
  aoai13aa1n04x5               g057(.a(new_n152), .b(new_n137), .c(new_n107), .d(new_n117), .o1(new_n153));
  tech160nm_fioai012aa1n04x5   g058(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .o1(new_n154));
  nanb03aa1n12x5               g059(.a(new_n147), .b(new_n148), .c(new_n142), .out0(new_n155));
  tech160nm_fioai012aa1n05x5   g060(.a(new_n148), .b(new_n147), .c(new_n141), .o1(new_n156));
  oai013aa1n09x5               g061(.a(new_n156), .b(new_n134), .c(new_n155), .d(new_n154), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  nanp02aa1n06x5               g063(.a(new_n153), .b(new_n158), .o1(new_n159));
  nor002aa1n20x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nanp02aa1n04x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  nona22aa1n06x5               g067(.a(new_n135), .b(new_n155), .c(new_n154), .out0(new_n163));
  nano22aa1n02x4               g068(.a(new_n162), .b(new_n163), .c(new_n156), .out0(new_n164));
  aoi022aa1n02x5               g069(.a(new_n159), .b(new_n162), .c(new_n153), .d(new_n164), .o1(\s[13] ));
  inv000aa1d42x5               g070(.a(new_n160), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(new_n159), .b(new_n162), .o1(new_n167));
  nor002aa1n04x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand02aa1n06x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  norb03aa1n03x5               g075(.a(new_n169), .b(new_n160), .c(new_n168), .out0(new_n171));
  nanp02aa1n02x5               g076(.a(new_n167), .b(new_n171), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n170), .c(new_n166), .d(new_n167), .o1(\s[14] ));
  oaoi03aa1n02x5               g078(.a(\a[14] ), .b(\b[13] ), .c(new_n166), .o1(new_n174));
  nano23aa1n06x5               g079(.a(new_n160), .b(new_n168), .c(new_n169), .d(new_n161), .out0(new_n175));
  tech160nm_fixorc02aa1n05x5   g080(.a(\a[15] ), .b(\b[14] ), .out0(new_n176));
  aoai13aa1n06x5               g081(.a(new_n176), .b(new_n174), .c(new_n159), .d(new_n175), .o1(new_n177));
  aoi112aa1n02x5               g082(.a(new_n174), .b(new_n176), .c(new_n159), .d(new_n175), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n177), .b(new_n178), .out0(\s[15] ));
  nor042aa1n04x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n180), .o1(new_n181));
  xorc02aa1n12x5               g086(.a(\a[16] ), .b(\b[15] ), .out0(new_n182));
  inv000aa1d42x5               g087(.a(\a[16] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\b[15] ), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(new_n184), .b(new_n183), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(\b[15] ), .b(\a[16] ), .o1(new_n186));
  nano22aa1n02x4               g091(.a(new_n180), .b(new_n185), .c(new_n186), .out0(new_n187));
  nanp02aa1n03x5               g092(.a(new_n177), .b(new_n187), .o1(new_n188));
  aoai13aa1n02x5               g093(.a(new_n188), .b(new_n182), .c(new_n181), .d(new_n177), .o1(\s[16] ));
  nand23aa1n06x5               g094(.a(new_n175), .b(new_n176), .c(new_n182), .o1(new_n190));
  nano32aa1d12x5               g095(.a(new_n190), .b(new_n149), .c(new_n138), .d(new_n143), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n137), .c(new_n107), .d(new_n117), .o1(new_n192));
  oai012aa1n02x5               g097(.a(new_n169), .b(\b[14] ), .c(\a[15] ), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[14] ), .b(\a[15] ), .o1(new_n194));
  nanp03aa1n02x5               g099(.a(new_n185), .b(new_n194), .c(new_n186), .o1(new_n195));
  tech160nm_fioaoi03aa1n03p5x5 g100(.a(new_n183), .b(new_n184), .c(new_n180), .o1(new_n196));
  oai013aa1n03x4               g101(.a(new_n196), .b(new_n171), .c(new_n195), .d(new_n193), .o1(new_n197));
  aoib12aa1n06x5               g102(.a(new_n197), .b(new_n157), .c(new_n190), .out0(new_n198));
  nanp02aa1n06x5               g103(.a(new_n192), .b(new_n198), .o1(new_n199));
  tech160nm_fixorc02aa1n03p5x5 g104(.a(\a[17] ), .b(\b[16] ), .out0(new_n200));
  norb02aa1n02x5               g105(.a(new_n196), .b(new_n200), .out0(new_n201));
  oai013aa1n02x4               g106(.a(new_n201), .b(new_n193), .c(new_n171), .d(new_n195), .o1(new_n202));
  aoib12aa1n02x5               g107(.a(new_n202), .b(new_n157), .c(new_n190), .out0(new_n203));
  aoi022aa1n02x5               g108(.a(new_n199), .b(new_n200), .c(new_n192), .d(new_n203), .o1(\s[17] ));
  inv000aa1d42x5               g109(.a(\a[17] ), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(\b[16] ), .b(new_n205), .out0(new_n206));
  inv030aa1n02x5               g111(.a(new_n197), .o1(new_n207));
  aoai13aa1n12x5               g112(.a(new_n207), .b(new_n190), .c(new_n163), .d(new_n156), .o1(new_n208));
  aoai13aa1n03x5               g113(.a(new_n200), .b(new_n208), .c(new_n129), .d(new_n191), .o1(new_n209));
  tech160nm_fixorc02aa1n02p5x5 g114(.a(\a[18] ), .b(\b[17] ), .out0(new_n210));
  and002aa1n06x5               g115(.a(\b[17] ), .b(\a[18] ), .o(new_n211));
  oai022aa1d18x5               g116(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n212));
  nona22aa1n02x4               g117(.a(new_n209), .b(new_n211), .c(new_n212), .out0(new_n213));
  aoai13aa1n02x5               g118(.a(new_n213), .b(new_n210), .c(new_n206), .d(new_n209), .o1(\s[18] ));
  and002aa1n02x5               g119(.a(new_n210), .b(new_n200), .o(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n208), .c(new_n129), .d(new_n191), .o1(new_n216));
  oaoi03aa1n02x5               g121(.a(\a[18] ), .b(\b[17] ), .c(new_n206), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  tech160nm_fixorc02aa1n03p5x5 g123(.a(\a[19] ), .b(\b[18] ), .out0(new_n219));
  xnbna2aa1n03x5               g124(.a(new_n219), .b(new_n216), .c(new_n218), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g126(.a(\a[19] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[18] ), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(new_n223), .b(new_n222), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n219), .b(new_n217), .c(new_n199), .d(new_n215), .o1(new_n225));
  norp02aa1n09x5               g130(.a(\b[19] ), .b(\a[20] ), .o1(new_n226));
  nand42aa1d28x5               g131(.a(\b[19] ), .b(\a[20] ), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n227), .b(new_n226), .out0(new_n228));
  inv000aa1n02x5               g133(.a(new_n219), .o1(new_n229));
  nano22aa1n02x4               g134(.a(new_n226), .b(new_n224), .c(new_n227), .out0(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n229), .c(new_n216), .d(new_n218), .o1(new_n231));
  aoai13aa1n03x5               g136(.a(new_n231), .b(new_n228), .c(new_n225), .d(new_n224), .o1(\s[20] ));
  nano32aa1n03x7               g137(.a(new_n229), .b(new_n210), .c(new_n200), .d(new_n228), .out0(new_n233));
  aoai13aa1n04x5               g138(.a(new_n233), .b(new_n208), .c(new_n129), .d(new_n191), .o1(new_n234));
  aoi022aa1n06x5               g139(.a(new_n223), .b(new_n222), .c(\a[18] ), .d(\b[17] ), .o1(new_n235));
  nand42aa1n06x5               g140(.a(\b[18] ), .b(\a[19] ), .o1(new_n236));
  nano22aa1n03x7               g141(.a(new_n226), .b(new_n236), .c(new_n227), .out0(new_n237));
  oai112aa1n06x5               g142(.a(new_n237), .b(new_n235), .c(new_n212), .d(new_n211), .o1(new_n238));
  aoai13aa1n04x5               g143(.a(new_n227), .b(new_n226), .c(new_n222), .d(new_n223), .o1(new_n239));
  nand02aa1d06x5               g144(.a(new_n238), .b(new_n239), .o1(new_n240));
  nor002aa1d32x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  nand02aa1d28x5               g146(.a(\b[20] ), .b(\a[21] ), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n240), .c(new_n199), .d(new_n233), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n243), .o1(new_n245));
  and003aa1n02x5               g150(.a(new_n238), .b(new_n245), .c(new_n239), .o(new_n246));
  aobi12aa1n02x7               g151(.a(new_n244), .b(new_n246), .c(new_n234), .out0(\s[21] ));
  inv000aa1d42x5               g152(.a(new_n241), .o1(new_n248));
  nor002aa1d32x5               g153(.a(\b[21] ), .b(\a[22] ), .o1(new_n249));
  nand42aa1d28x5               g154(.a(\b[21] ), .b(\a[22] ), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n250), .b(new_n249), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n240), .o1(new_n252));
  nona22aa1d36x5               g157(.a(new_n250), .b(new_n249), .c(new_n241), .out0(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  aoai13aa1n02x7               g159(.a(new_n254), .b(new_n245), .c(new_n234), .d(new_n252), .o1(new_n255));
  aoai13aa1n03x5               g160(.a(new_n255), .b(new_n251), .c(new_n244), .d(new_n248), .o1(\s[22] ));
  nano32aa1n02x4               g161(.a(new_n226), .b(new_n224), .c(new_n227), .d(new_n236), .out0(new_n257));
  nano23aa1d15x5               g162(.a(new_n241), .b(new_n249), .c(new_n250), .d(new_n242), .out0(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  nano32aa1n02x4               g164(.a(new_n259), .b(new_n257), .c(new_n210), .d(new_n200), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n208), .c(new_n129), .d(new_n191), .o1(new_n261));
  aoi022aa1n02x5               g166(.a(new_n240), .b(new_n258), .c(new_n250), .d(new_n253), .o1(new_n262));
  inv030aa1n02x5               g167(.a(new_n262), .o1(new_n263));
  nor042aa1n09x5               g168(.a(\b[22] ), .b(\a[23] ), .o1(new_n264));
  nand22aa1n12x5               g169(.a(\b[22] ), .b(\a[23] ), .o1(new_n265));
  norb02aa1n12x5               g170(.a(new_n265), .b(new_n264), .out0(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n263), .c(new_n199), .d(new_n260), .o1(new_n267));
  aoi122aa1n02x5               g172(.a(new_n266), .b(new_n250), .c(new_n253), .d(new_n240), .e(new_n258), .o1(new_n268));
  aobi12aa1n02x7               g173(.a(new_n267), .b(new_n268), .c(new_n261), .out0(\s[23] ));
  inv000aa1d42x5               g174(.a(new_n264), .o1(new_n270));
  xorc02aa1n12x5               g175(.a(\a[24] ), .b(\b[23] ), .out0(new_n271));
  inv000aa1d42x5               g176(.a(new_n266), .o1(new_n272));
  and002aa1n12x5               g177(.a(\b[23] ), .b(\a[24] ), .o(new_n273));
  oai022aa1n06x5               g178(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n274));
  norp02aa1n02x5               g179(.a(new_n274), .b(new_n273), .o1(new_n275));
  aoai13aa1n02x7               g180(.a(new_n275), .b(new_n272), .c(new_n261), .d(new_n262), .o1(new_n276));
  aoai13aa1n02x5               g181(.a(new_n276), .b(new_n271), .c(new_n267), .d(new_n270), .o1(\s[24] ));
  nand23aa1d12x5               g182(.a(new_n258), .b(new_n266), .c(new_n271), .o1(new_n278));
  nano32aa1n02x4               g183(.a(new_n278), .b(new_n257), .c(new_n210), .d(new_n200), .out0(new_n279));
  aoai13aa1n03x5               g184(.a(new_n279), .b(new_n208), .c(new_n129), .d(new_n191), .o1(new_n280));
  inv000aa1n02x5               g185(.a(new_n273), .o1(new_n281));
  oai012aa1n06x5               g186(.a(new_n250), .b(\b[22] ), .c(\a[23] ), .o1(new_n282));
  oai012aa1n02x5               g187(.a(new_n265), .b(\b[23] ), .c(\a[24] ), .o1(new_n283));
  nor003aa1n03x5               g188(.a(new_n283), .b(new_n282), .c(new_n273), .o1(new_n284));
  aoi022aa1d18x5               g189(.a(new_n284), .b(new_n253), .c(new_n281), .d(new_n274), .o1(new_n285));
  aoai13aa1n12x5               g190(.a(new_n285), .b(new_n278), .c(new_n238), .d(new_n239), .o1(new_n286));
  xorc02aa1n12x5               g191(.a(\a[25] ), .b(\b[24] ), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n286), .c(new_n199), .d(new_n279), .o1(new_n288));
  inv040aa1n02x5               g193(.a(new_n278), .o1(new_n289));
  aoi122aa1n02x5               g194(.a(new_n287), .b(new_n281), .c(new_n274), .d(new_n284), .e(new_n253), .o1(new_n290));
  aobi12aa1n02x5               g195(.a(new_n290), .b(new_n289), .c(new_n240), .out0(new_n291));
  aobi12aa1n02x5               g196(.a(new_n288), .b(new_n291), .c(new_n280), .out0(\s[25] ));
  orn002aa1n02x5               g197(.a(\a[25] ), .b(\b[24] ), .o(new_n293));
  xorc02aa1n02x5               g198(.a(\a[26] ), .b(\b[25] ), .out0(new_n294));
  inv000aa1d42x5               g199(.a(new_n286), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n287), .o1(new_n296));
  and002aa1n02x5               g201(.a(\b[25] ), .b(\a[26] ), .o(new_n297));
  oai022aa1n02x5               g202(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n298));
  norp02aa1n02x5               g203(.a(new_n298), .b(new_n297), .o1(new_n299));
  aoai13aa1n02x7               g204(.a(new_n299), .b(new_n296), .c(new_n280), .d(new_n295), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n294), .c(new_n288), .d(new_n293), .o1(\s[26] ));
  and002aa1n02x5               g206(.a(new_n287), .b(new_n294), .o(new_n302));
  nand03aa1n02x5               g207(.a(new_n289), .b(new_n233), .c(new_n302), .o1(new_n303));
  inv000aa1n02x5               g208(.a(new_n303), .o1(new_n304));
  aoai13aa1n02x7               g209(.a(new_n304), .b(new_n208), .c(new_n129), .d(new_n191), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n297), .o1(new_n306));
  aoi022aa1n12x5               g211(.a(new_n286), .b(new_n302), .c(new_n306), .d(new_n298), .o1(new_n307));
  aoai13aa1n06x5               g212(.a(new_n307), .b(new_n303), .c(new_n192), .d(new_n198), .o1(new_n308));
  xorc02aa1n12x5               g213(.a(\a[27] ), .b(\b[26] ), .out0(new_n309));
  aoi122aa1n02x5               g214(.a(new_n309), .b(new_n306), .c(new_n298), .d(new_n286), .e(new_n302), .o1(new_n310));
  aoi022aa1n03x5               g215(.a(new_n308), .b(new_n309), .c(new_n305), .d(new_n310), .o1(\s[27] ));
  norp02aa1n02x5               g216(.a(\b[26] ), .b(\a[27] ), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n312), .o1(new_n313));
  nand42aa1n02x5               g218(.a(new_n308), .b(new_n309), .o1(new_n314));
  xorc02aa1n02x5               g219(.a(\a[28] ), .b(\b[27] ), .out0(new_n315));
  inv000aa1d42x5               g220(.a(new_n309), .o1(new_n316));
  nand22aa1n03x5               g221(.a(\b[27] ), .b(\a[28] ), .o1(new_n317));
  oai022aa1d18x5               g222(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n318));
  norb02aa1n06x4               g223(.a(new_n317), .b(new_n318), .out0(new_n319));
  aoai13aa1n02x5               g224(.a(new_n319), .b(new_n316), .c(new_n305), .d(new_n307), .o1(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n315), .c(new_n314), .d(new_n313), .o1(\s[28] ));
  nor042aa1n02x5               g226(.a(\b[28] ), .b(\a[29] ), .o1(new_n322));
  nanp02aa1n02x5               g227(.a(\b[28] ), .b(\a[29] ), .o1(new_n323));
  nanb02aa1n06x5               g228(.a(new_n322), .b(new_n323), .out0(new_n324));
  and002aa1n02x5               g229(.a(new_n315), .b(new_n309), .o(new_n325));
  oaoi03aa1n02x5               g230(.a(\a[28] ), .b(\b[27] ), .c(new_n313), .o1(new_n326));
  aoai13aa1n03x5               g231(.a(new_n324), .b(new_n326), .c(new_n308), .d(new_n325), .o1(new_n327));
  inv000aa1d42x5               g232(.a(new_n325), .o1(new_n328));
  aoi012aa1n02x5               g233(.a(new_n324), .b(new_n317), .c(new_n318), .o1(new_n329));
  aoai13aa1n02x5               g234(.a(new_n329), .b(new_n328), .c(new_n305), .d(new_n307), .o1(new_n330));
  nanp02aa1n03x5               g235(.a(new_n327), .b(new_n330), .o1(\s[29] ));
  xorb03aa1n02x5               g236(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g237(.a(new_n324), .b(new_n309), .c(new_n315), .out0(new_n333));
  nand42aa1n02x5               g238(.a(new_n308), .b(new_n333), .o1(new_n334));
  aoi013aa1n02x4               g239(.a(new_n322), .b(new_n318), .c(new_n317), .d(new_n323), .o1(new_n335));
  norp02aa1n02x5               g240(.a(\b[29] ), .b(\a[30] ), .o1(new_n336));
  nanp02aa1n02x5               g241(.a(\b[29] ), .b(\a[30] ), .o1(new_n337));
  norb02aa1n02x5               g242(.a(new_n337), .b(new_n336), .out0(new_n338));
  inv000aa1n02x5               g243(.a(new_n333), .o1(new_n339));
  nona23aa1n03x5               g244(.a(new_n323), .b(new_n317), .c(new_n319), .d(new_n322), .out0(new_n340));
  nona23aa1n09x5               g245(.a(new_n340), .b(new_n337), .c(new_n336), .d(new_n322), .out0(new_n341));
  inv000aa1d42x5               g246(.a(new_n341), .o1(new_n342));
  aoai13aa1n02x5               g247(.a(new_n342), .b(new_n339), .c(new_n305), .d(new_n307), .o1(new_n343));
  aoai13aa1n03x5               g248(.a(new_n343), .b(new_n338), .c(new_n334), .d(new_n335), .o1(\s[30] ));
  nano32aa1d12x5               g249(.a(new_n324), .b(new_n315), .c(new_n309), .d(new_n338), .out0(new_n345));
  and002aa1n02x5               g250(.a(new_n341), .b(new_n337), .o(new_n346));
  xnrc02aa1n02x5               g251(.a(\b[30] ), .b(\a[31] ), .out0(new_n347));
  aoai13aa1n03x5               g252(.a(new_n347), .b(new_n346), .c(new_n308), .d(new_n345), .o1(new_n348));
  inv000aa1d42x5               g253(.a(new_n345), .o1(new_n349));
  aoi012aa1n02x5               g254(.a(new_n347), .b(new_n341), .c(new_n337), .o1(new_n350));
  aoai13aa1n02x5               g255(.a(new_n350), .b(new_n349), .c(new_n305), .d(new_n307), .o1(new_n351));
  nanp02aa1n03x5               g256(.a(new_n348), .b(new_n351), .o1(\s[31] ));
  norb02aa1n02x5               g257(.a(new_n104), .b(new_n103), .out0(new_n353));
  aoi112aa1n02x5               g258(.a(new_n353), .b(new_n97), .c(new_n98), .d(new_n99), .o1(new_n354));
  aoib12aa1n02x5               g259(.a(new_n354), .b(new_n353), .c(new_n100), .out0(\s[3] ));
  norb02aa1n02x5               g260(.a(new_n102), .b(new_n101), .out0(new_n356));
  inv000aa1d42x5               g261(.a(new_n103), .o1(new_n357));
  aoai13aa1n02x5               g262(.a(new_n353), .b(new_n97), .c(new_n99), .d(new_n98), .o1(new_n358));
  xnbna2aa1n03x5               g263(.a(new_n356), .b(new_n358), .c(new_n357), .out0(\s[4] ));
  nanb03aa1n02x5               g264(.a(new_n100), .b(new_n353), .c(new_n356), .out0(new_n360));
  norb02aa1n02x5               g265(.a(new_n109), .b(new_n108), .out0(new_n361));
  xnbna2aa1n03x5               g266(.a(new_n361), .b(new_n360), .c(new_n106), .out0(\s[5] ));
  norb02aa1n02x5               g267(.a(new_n111), .b(new_n110), .out0(new_n363));
  aoi012aa1n02x5               g268(.a(new_n108), .b(new_n107), .c(new_n109), .o1(new_n364));
  aob012aa1n02x5               g269(.a(new_n119), .b(new_n107), .c(new_n361), .out0(new_n365));
  oai012aa1n02x5               g270(.a(new_n365), .b(new_n364), .c(new_n363), .o1(\s[6] ));
  xnbna2aa1n03x5               g271(.a(new_n116), .b(new_n365), .c(new_n111), .out0(\s[7] ));
  nanb02aa1n02x5               g272(.a(new_n120), .b(new_n365), .out0(new_n368));
  xobna2aa1n03x5               g273(.a(new_n113), .b(new_n368), .c(new_n122), .out0(\s[8] ));
  norb03aa1n02x5               g274(.a(new_n123), .b(new_n121), .c(new_n127), .out0(new_n370));
  aoi022aa1n02x5               g275(.a(new_n129), .b(new_n127), .c(new_n118), .d(new_n370), .o1(\s[9] ));
endmodule

