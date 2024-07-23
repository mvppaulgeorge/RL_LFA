// Benchmark "adder" written by ABC on Wed Jul 17 20:44:29 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n329, new_n331,
    new_n332, new_n333, new_n334, new_n337, new_n339, new_n340, new_n341,
    new_n342, new_n344;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand42aa1n08x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand42aa1n03x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nor002aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nona22aa1n06x5               g007(.a(new_n101), .b(new_n102), .c(new_n100), .out0(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nano22aa1n06x5               g010(.a(new_n104), .b(new_n101), .c(new_n105), .out0(new_n106));
  orn002aa1n24x5               g011(.a(\a[4] ), .b(\b[3] ), .o(new_n107));
  oai112aa1n06x5               g012(.a(new_n107), .b(new_n99), .c(\b[2] ), .d(\a[3] ), .o1(new_n108));
  aoai13aa1n12x5               g013(.a(new_n99), .b(new_n108), .c(new_n106), .d(new_n103), .o1(new_n109));
  nor002aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor042aa1n09x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand42aa1n03x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n03x7               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  xorc02aa1n02x5               g019(.a(\a[6] ), .b(\b[5] ), .out0(new_n115));
  xnrc02aa1n12x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  inv000aa1d42x5               g021(.a(new_n116), .o1(new_n117));
  nand23aa1n03x5               g022(.a(new_n117), .b(new_n114), .c(new_n115), .o1(new_n118));
  inv000aa1d42x5               g023(.a(new_n112), .o1(new_n119));
  oaoi03aa1n02x5               g024(.a(\a[8] ), .b(\b[7] ), .c(new_n119), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[6] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[5] ), .o1(new_n122));
  nor042aa1n02x5               g027(.a(\b[4] ), .b(\a[5] ), .o1(new_n123));
  oao003aa1n02x5               g028(.a(new_n121), .b(new_n122), .c(new_n123), .carry(new_n124));
  tech160nm_fiaoi012aa1n03p5x5 g029(.a(new_n120), .b(new_n114), .c(new_n124), .o1(new_n125));
  tech160nm_fioai012aa1n05x5   g030(.a(new_n125), .b(new_n109), .c(new_n118), .o1(new_n126));
  nanp02aa1n04x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nanp03aa1n02x5               g032(.a(new_n126), .b(new_n98), .c(new_n127), .o1(new_n128));
  nor022aa1n12x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nanp02aa1n12x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  nona23aa1n02x4               g036(.a(new_n128), .b(new_n130), .c(new_n129), .d(new_n97), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n131), .c(new_n98), .d(new_n128), .o1(\s[10] ));
  nona23aa1d24x5               g038(.a(new_n130), .b(new_n127), .c(new_n97), .d(new_n129), .out0(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  tech160nm_fioai012aa1n05x5   g040(.a(new_n130), .b(new_n129), .c(new_n97), .o1(new_n136));
  inv040aa1n03x5               g041(.a(new_n136), .o1(new_n137));
  tech160nm_fiaoi012aa1n05x5   g042(.a(new_n137), .b(new_n126), .c(new_n135), .o1(new_n138));
  nor002aa1d32x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  inv040aa1n03x5               g044(.a(new_n139), .o1(new_n140));
  nand02aa1n03x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n138), .b(new_n141), .c(new_n140), .out0(\s[11] ));
  nanb02aa1n02x5               g047(.a(new_n139), .b(new_n141), .out0(new_n143));
  oab012aa1n02x4               g048(.a(new_n139), .b(new_n138), .c(new_n143), .out0(new_n144));
  orn002aa1n02x5               g049(.a(\a[12] ), .b(\b[11] ), .o(new_n145));
  nand42aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nor002aa1n03x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nona22aa1n02x4               g052(.a(new_n146), .b(new_n147), .c(new_n139), .out0(new_n148));
  oabi12aa1n02x5               g053(.a(new_n148), .b(new_n138), .c(new_n143), .out0(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n144), .c(new_n146), .d(new_n145), .o1(\s[12] ));
  nano23aa1n06x5               g055(.a(new_n139), .b(new_n147), .c(new_n146), .d(new_n141), .out0(new_n151));
  nanb02aa1n02x5               g056(.a(new_n134), .b(new_n151), .out0(new_n152));
  nona23aa1n09x5               g057(.a(new_n146), .b(new_n141), .c(new_n139), .d(new_n147), .out0(new_n153));
  oaoi03aa1n09x5               g058(.a(\a[12] ), .b(\b[11] ), .c(new_n140), .o1(new_n154));
  oabi12aa1n12x5               g059(.a(new_n154), .b(new_n153), .c(new_n136), .out0(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  oaib12aa1n03x5               g061(.a(new_n156), .b(new_n152), .c(new_n126), .out0(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  tech160nm_fixnrc02aa1n04x5   g063(.a(\b[12] ), .b(\a[13] ), .out0(new_n159));
  norb02aa1n02x5               g064(.a(new_n157), .b(new_n159), .out0(new_n160));
  norp02aa1n02x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  aoib12aa1n02x5               g066(.a(new_n161), .b(new_n157), .c(new_n159), .out0(new_n162));
  tech160nm_fixnrc02aa1n04x5   g067(.a(\b[13] ), .b(\a[14] ), .out0(new_n163));
  nanp02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  oai022aa1n02x5               g069(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n165));
  nanb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  obai22aa1n03x5               g071(.a(new_n163), .b(new_n162), .c(new_n160), .d(new_n166), .out0(\s[14] ));
  nor042aa1n03x5               g072(.a(new_n163), .b(new_n159), .o1(new_n168));
  aoi022aa1n09x5               g073(.a(new_n155), .b(new_n168), .c(new_n164), .d(new_n165), .o1(new_n169));
  nona32aa1n03x5               g074(.a(new_n126), .b(new_n163), .c(new_n159), .d(new_n152), .out0(new_n170));
  nor042aa1n06x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  tech160nm_finand02aa1n03p5x5 g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n173), .b(new_n170), .c(new_n169), .out0(\s[15] ));
  inv000aa1d42x5               g079(.a(new_n171), .o1(new_n175));
  aob012aa1n03x5               g080(.a(new_n173), .b(new_n170), .c(new_n169), .out0(new_n176));
  nor002aa1n03x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand42aa1n06x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  nona23aa1n03x5               g084(.a(new_n176), .b(new_n178), .c(new_n177), .d(new_n171), .out0(new_n180));
  aoai13aa1n03x5               g085(.a(new_n180), .b(new_n179), .c(new_n175), .d(new_n176), .o1(\s[16] ));
  nano23aa1d15x5               g086(.a(new_n171), .b(new_n177), .c(new_n178), .d(new_n172), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n182), .o1(new_n183));
  nona23aa1n09x5               g088(.a(new_n168), .b(new_n182), .c(new_n153), .d(new_n134), .out0(new_n184));
  inv000aa1n02x5               g089(.a(new_n184), .o1(new_n185));
  nand42aa1n04x5               g090(.a(new_n126), .b(new_n185), .o1(new_n186));
  oai012aa1n02x5               g091(.a(new_n178), .b(new_n177), .c(new_n171), .o1(new_n187));
  oai112aa1n06x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .d(new_n169), .o1(new_n188));
  xorb03aa1n03x5               g093(.a(new_n188), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1d18x5               g094(.a(\b[16] ), .b(\a[17] ), .o1(new_n190));
  inv040aa1n08x5               g095(.a(new_n190), .o1(new_n191));
  oaoi13aa1n09x5               g096(.a(new_n184), .b(new_n125), .c(new_n109), .d(new_n118), .o1(new_n192));
  aoai13aa1n02x5               g097(.a(new_n168), .b(new_n154), .c(new_n151), .d(new_n137), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(new_n165), .b(new_n164), .o1(new_n194));
  aoai13aa1n06x5               g099(.a(new_n187), .b(new_n183), .c(new_n193), .d(new_n194), .o1(new_n195));
  xorc02aa1n02x5               g100(.a(\a[17] ), .b(\b[16] ), .out0(new_n196));
  oai012aa1n02x5               g101(.a(new_n196), .b(new_n195), .c(new_n192), .o1(new_n197));
  xorc02aa1n03x5               g102(.a(\a[18] ), .b(\b[17] ), .out0(new_n198));
  inv000aa1d42x5               g103(.a(\a[18] ), .o1(new_n199));
  inv000aa1d42x5               g104(.a(\b[17] ), .o1(new_n200));
  aoi012aa1n02x5               g105(.a(new_n190), .b(new_n199), .c(new_n200), .o1(new_n201));
  oai112aa1n02x5               g106(.a(new_n197), .b(new_n201), .c(new_n200), .d(new_n199), .o1(new_n202));
  aoai13aa1n02x5               g107(.a(new_n202), .b(new_n198), .c(new_n191), .d(new_n197), .o1(\s[18] ));
  and002aa1n02x5               g108(.a(new_n198), .b(new_n196), .o(new_n204));
  oaih12aa1n02x5               g109(.a(new_n204), .b(new_n195), .c(new_n192), .o1(new_n205));
  oaoi03aa1n12x5               g110(.a(\a[18] ), .b(\b[17] ), .c(new_n191), .o1(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  nor042aa1n09x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nand42aa1n02x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nanb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n205), .c(new_n207), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g118(.a(new_n208), .o1(new_n214));
  aoai13aa1n03x5               g119(.a(new_n211), .b(new_n206), .c(new_n188), .d(new_n204), .o1(new_n215));
  nor042aa1n02x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nand02aa1n12x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n217), .b(new_n216), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n217), .o1(new_n219));
  oai022aa1n02x5               g124(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n220));
  norp02aa1n02x5               g125(.a(new_n220), .b(new_n219), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n221), .b(new_n210), .c(new_n205), .d(new_n207), .o1(new_n222));
  aoai13aa1n03x5               g127(.a(new_n222), .b(new_n218), .c(new_n215), .d(new_n214), .o1(\s[20] ));
  nano23aa1n09x5               g128(.a(new_n208), .b(new_n216), .c(new_n217), .d(new_n209), .out0(new_n224));
  nand23aa1n04x5               g129(.a(new_n224), .b(new_n196), .c(new_n198), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  oaih12aa1n02x5               g131(.a(new_n226), .b(new_n195), .c(new_n192), .o1(new_n227));
  aoi022aa1n02x7               g132(.a(new_n224), .b(new_n206), .c(new_n217), .d(new_n220), .o1(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[20] ), .b(\a[21] ), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  xnbna2aa1n03x5               g135(.a(new_n230), .b(new_n227), .c(new_n228), .out0(\s[21] ));
  norp02aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n228), .o1(new_n234));
  aoai13aa1n04x5               g139(.a(new_n230), .b(new_n234), .c(new_n188), .d(new_n226), .o1(new_n235));
  xnrc02aa1n12x5               g140(.a(\b[21] ), .b(\a[22] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  nanp02aa1n02x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  oai022aa1n02x5               g143(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n238), .b(new_n239), .out0(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n229), .c(new_n227), .d(new_n228), .o1(new_n241));
  aoai13aa1n03x5               g146(.a(new_n241), .b(new_n237), .c(new_n235), .d(new_n233), .o1(\s[22] ));
  nor002aa1n04x5               g147(.a(new_n236), .b(new_n229), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n225), .out0(new_n244));
  oaih12aa1n02x5               g149(.a(new_n244), .b(new_n195), .c(new_n192), .o1(new_n245));
  oab012aa1n02x4               g150(.a(new_n219), .b(new_n208), .c(new_n216), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n243), .b(new_n246), .c(new_n224), .d(new_n206), .o1(new_n247));
  nanp02aa1n02x5               g152(.a(new_n239), .b(new_n238), .o1(new_n248));
  nanp02aa1n02x5               g153(.a(new_n247), .b(new_n248), .o1(new_n249));
  inv000aa1n02x5               g154(.a(new_n249), .o1(new_n250));
  xnrc02aa1n12x5               g155(.a(\b[22] ), .b(\a[23] ), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  xnbna2aa1n03x5               g157(.a(new_n252), .b(new_n245), .c(new_n250), .out0(\s[23] ));
  norp02aa1n02x5               g158(.a(\b[22] ), .b(\a[23] ), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  aoai13aa1n03x5               g160(.a(new_n252), .b(new_n249), .c(new_n188), .d(new_n244), .o1(new_n256));
  tech160nm_fixorc02aa1n02p5x5 g161(.a(\a[24] ), .b(\b[23] ), .out0(new_n257));
  nanp02aa1n02x5               g162(.a(\b[23] ), .b(\a[24] ), .o1(new_n258));
  oai022aa1n02x5               g163(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n259));
  norb02aa1n02x5               g164(.a(new_n258), .b(new_n259), .out0(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n251), .c(new_n245), .d(new_n250), .o1(new_n261));
  aoai13aa1n03x5               g166(.a(new_n261), .b(new_n257), .c(new_n256), .d(new_n255), .o1(\s[24] ));
  nano32aa1n03x7               g167(.a(new_n225), .b(new_n257), .c(new_n243), .d(new_n252), .out0(new_n263));
  oai012aa1n02x5               g168(.a(new_n263), .b(new_n195), .c(new_n192), .o1(new_n264));
  norb02aa1n02x5               g169(.a(new_n257), .b(new_n251), .out0(new_n265));
  inv000aa1n02x5               g170(.a(new_n265), .o1(new_n266));
  nanp02aa1n02x5               g171(.a(new_n259), .b(new_n258), .o1(new_n267));
  aoai13aa1n12x5               g172(.a(new_n267), .b(new_n266), .c(new_n247), .d(new_n248), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  xorc02aa1n12x5               g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  xnbna2aa1n03x5               g175(.a(new_n270), .b(new_n264), .c(new_n269), .out0(\s[25] ));
  norp02aa1n02x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  aoai13aa1n02x7               g178(.a(new_n270), .b(new_n268), .c(new_n188), .d(new_n263), .o1(new_n274));
  xorc02aa1n02x5               g179(.a(\a[26] ), .b(\b[25] ), .out0(new_n275));
  inv000aa1d42x5               g180(.a(new_n270), .o1(new_n276));
  nanp02aa1n02x5               g181(.a(\b[25] ), .b(\a[26] ), .o1(new_n277));
  oai022aa1n02x5               g182(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n278));
  norb02aa1n02x5               g183(.a(new_n277), .b(new_n278), .out0(new_n279));
  aoai13aa1n02x5               g184(.a(new_n279), .b(new_n276), .c(new_n264), .d(new_n269), .o1(new_n280));
  aoai13aa1n03x5               g185(.a(new_n280), .b(new_n275), .c(new_n274), .d(new_n273), .o1(\s[26] ));
  and002aa1n02x5               g186(.a(new_n275), .b(new_n270), .o(new_n282));
  nano32aa1n03x7               g187(.a(new_n225), .b(new_n282), .c(new_n243), .d(new_n265), .out0(new_n283));
  oai012aa1n09x5               g188(.a(new_n283), .b(new_n195), .c(new_n192), .o1(new_n284));
  aoi022aa1d18x5               g189(.a(new_n268), .b(new_n282), .c(new_n277), .d(new_n278), .o1(new_n285));
  xorc02aa1n02x5               g190(.a(\a[27] ), .b(\b[26] ), .out0(new_n286));
  xnbna2aa1n03x5               g191(.a(new_n286), .b(new_n284), .c(new_n285), .out0(\s[27] ));
  norp02aa1n02x5               g192(.a(\b[26] ), .b(\a[27] ), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n288), .o1(new_n289));
  nanp02aa1n03x5               g194(.a(new_n268), .b(new_n282), .o1(new_n290));
  nanp02aa1n02x5               g195(.a(new_n278), .b(new_n277), .o1(new_n291));
  nand42aa1n02x5               g196(.a(new_n290), .b(new_n291), .o1(new_n292));
  aoai13aa1n02x7               g197(.a(new_n286), .b(new_n292), .c(new_n188), .d(new_n283), .o1(new_n293));
  xorc02aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .out0(new_n294));
  inv000aa1n02x5               g199(.a(new_n286), .o1(new_n295));
  oai022aa1n02x5               g200(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n296));
  aoi012aa1n02x5               g201(.a(new_n296), .b(\a[28] ), .c(\b[27] ), .o1(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n295), .c(new_n284), .d(new_n285), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n294), .c(new_n293), .d(new_n289), .o1(\s[28] ));
  and002aa1n02x5               g204(.a(new_n294), .b(new_n286), .o(new_n300));
  inv000aa1d42x5               g205(.a(new_n300), .o1(new_n301));
  aob012aa1n02x5               g206(.a(new_n296), .b(\b[27] ), .c(\a[28] ), .out0(new_n302));
  xorc02aa1n12x5               g207(.a(\a[29] ), .b(\b[28] ), .out0(new_n303));
  and002aa1n02x5               g208(.a(new_n303), .b(new_n302), .o(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n301), .c(new_n284), .d(new_n285), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n303), .o1(new_n306));
  aoai13aa1n06x5               g211(.a(new_n302), .b(new_n301), .c(new_n284), .d(new_n285), .o1(new_n307));
  nanp02aa1n03x5               g212(.a(new_n307), .b(new_n306), .o1(new_n308));
  nanp02aa1n03x5               g213(.a(new_n308), .b(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g214(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanp03aa1n02x5               g215(.a(new_n294), .b(new_n286), .c(new_n303), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[29] ), .b(\b[28] ), .c(new_n302), .carry(new_n312));
  aoai13aa1n06x5               g217(.a(new_n312), .b(new_n311), .c(new_n284), .d(new_n285), .o1(new_n313));
  xorc02aa1n12x5               g218(.a(\a[30] ), .b(\b[29] ), .out0(new_n314));
  inv000aa1d42x5               g219(.a(new_n314), .o1(new_n315));
  nanp02aa1n03x5               g220(.a(new_n313), .b(new_n315), .o1(new_n316));
  norb02aa1n02x5               g221(.a(new_n312), .b(new_n315), .out0(new_n317));
  aoai13aa1n02x5               g222(.a(new_n317), .b(new_n311), .c(new_n284), .d(new_n285), .o1(new_n318));
  nanp02aa1n03x5               g223(.a(new_n316), .b(new_n318), .o1(\s[30] ));
  nano32aa1n03x7               g224(.a(new_n295), .b(new_n314), .c(new_n294), .d(new_n303), .out0(new_n320));
  aoai13aa1n02x7               g225(.a(new_n320), .b(new_n292), .c(new_n188), .d(new_n283), .o1(new_n321));
  xorc02aa1n02x5               g226(.a(\a[31] ), .b(\b[30] ), .out0(new_n322));
  inv000aa1n02x5               g227(.a(new_n320), .o1(new_n323));
  oai012aa1n02x5               g228(.a(new_n322), .b(\b[29] ), .c(\a[30] ), .o1(new_n324));
  oab012aa1n02x4               g229(.a(new_n324), .b(new_n312), .c(new_n315), .out0(new_n325));
  aoai13aa1n03x5               g230(.a(new_n325), .b(new_n323), .c(new_n284), .d(new_n285), .o1(new_n326));
  oao003aa1n02x5               g231(.a(\a[30] ), .b(\b[29] ), .c(new_n312), .carry(new_n327));
  aoai13aa1n03x5               g232(.a(new_n326), .b(new_n322), .c(new_n321), .d(new_n327), .o1(\s[31] ));
  norb02aa1n02x5               g233(.a(new_n105), .b(new_n104), .out0(new_n329));
  xobna2aa1n03x5               g234(.a(new_n329), .b(new_n103), .c(new_n101), .out0(\s[3] ));
  oai112aa1n02x5               g235(.a(new_n329), .b(new_n101), .c(new_n102), .d(new_n100), .o1(new_n331));
  inv000aa1d42x5               g236(.a(new_n108), .o1(new_n332));
  nanp02aa1n02x5               g237(.a(new_n331), .b(new_n332), .o1(new_n333));
  aoi012aa1n02x5               g238(.a(new_n104), .b(new_n106), .c(new_n103), .o1(new_n334));
  aoai13aa1n02x5               g239(.a(new_n333), .b(new_n334), .c(new_n107), .d(new_n99), .o1(\s[4] ));
  xnrc02aa1n02x5               g240(.a(new_n109), .b(new_n117), .out0(\s[5] ));
  aoi013aa1n02x4               g241(.a(new_n123), .b(new_n333), .c(new_n99), .d(new_n117), .o1(new_n337));
  xorb03aa1n02x5               g242(.a(new_n337), .b(\b[5] ), .c(new_n121), .out0(\s[6] ));
  norb02aa1n02x5               g243(.a(new_n113), .b(new_n112), .out0(new_n339));
  nanp02aa1n02x5               g244(.a(new_n337), .b(new_n115), .o1(new_n340));
  oai112aa1n02x5               g245(.a(new_n340), .b(new_n339), .c(new_n122), .d(new_n121), .o1(new_n341));
  oaoi13aa1n02x5               g246(.a(new_n339), .b(new_n340), .c(new_n121), .d(new_n122), .o1(new_n342));
  norb02aa1n02x5               g247(.a(new_n341), .b(new_n342), .out0(\s[7] ));
  norb02aa1n02x5               g248(.a(new_n111), .b(new_n110), .out0(new_n344));
  xnbna2aa1n03x5               g249(.a(new_n344), .b(new_n341), .c(new_n119), .out0(\s[8] ));
  xorb03aa1n02x5               g250(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


