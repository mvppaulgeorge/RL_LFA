// Benchmark "adder" written by ABC on Thu Jul 18 05:39:25 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n329, new_n330,
    new_n331, new_n333, new_n334, new_n336, new_n337, new_n339, new_n340,
    new_n341, new_n343, new_n345, new_n346, new_n347, new_n349, new_n350,
    new_n351;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1n02x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1n02x7               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  tech160nm_finor002aa1n03p5x5 g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor022aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n06x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  aoi012aa1n02x5               g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  tech160nm_fioai012aa1n05x5   g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  nor042aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nand22aa1n03x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  norp02aa1n04x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  nor042aa1n06x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nanb02aa1n02x5               g019(.a(new_n113), .b(new_n114), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor043aa1n02x5               g021(.a(new_n112), .b(new_n115), .c(new_n116), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n107), .b(new_n117), .o1(new_n118));
  oa0012aa1n02x5               g023(.a(new_n109), .b(new_n110), .c(new_n108), .o(new_n119));
  nor042aa1n03x5               g024(.a(\b[8] ), .b(\a[9] ), .o1(new_n120));
  nanb03aa1n02x5               g025(.a(new_n110), .b(new_n111), .c(new_n109), .out0(new_n121));
  nor042aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  oai122aa1n02x7               g027(.a(new_n114), .b(new_n113), .c(new_n122), .d(\b[7] ), .e(\a[8] ), .o1(new_n123));
  nor002aa1n02x5               g028(.a(new_n123), .b(new_n121), .o1(new_n124));
  nona32aa1n03x5               g029(.a(new_n118), .b(new_n124), .c(new_n120), .d(new_n119), .out0(new_n125));
  nand42aa1n04x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand42aa1n03x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  tech160nm_fioai012aa1n04x5   g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .o1(new_n128));
  nanb03aa1n06x5               g033(.a(new_n128), .b(new_n125), .c(new_n126), .out0(new_n129));
  norp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  aboi22aa1n03x5               g035(.a(new_n130), .b(new_n126), .c(new_n125), .d(new_n127), .out0(new_n131));
  norb02aa1n02x5               g036(.a(new_n129), .b(new_n131), .out0(\s[10] ));
  tech160nm_fiao0012aa1n02p5x5 g037(.a(new_n130), .b(new_n120), .c(new_n126), .o(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nor022aa1n12x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1n03x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1d21x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n129), .c(new_n134), .out0(\s[11] ));
  aob012aa1n02x5               g043(.a(new_n137), .b(new_n129), .c(new_n134), .out0(new_n139));
  inv000aa1d42x5               g044(.a(new_n135), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n137), .o1(new_n141));
  aoai13aa1n02x5               g046(.a(new_n140), .b(new_n141), .c(new_n129), .d(new_n134), .o1(new_n142));
  nor002aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand42aa1n03x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n03x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  aoib12aa1n02x5               g050(.a(new_n135), .b(new_n144), .c(new_n143), .out0(new_n146));
  aoi022aa1n02x5               g051(.a(new_n142), .b(new_n145), .c(new_n139), .d(new_n146), .o1(\s[12] ));
  oabi12aa1n06x5               g052(.a(new_n119), .b(new_n121), .c(new_n123), .out0(new_n148));
  tech160nm_fioai012aa1n04x5   g053(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .o1(new_n149));
  nona23aa1d18x5               g054(.a(new_n145), .b(new_n137), .c(new_n149), .d(new_n128), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  aoai13aa1n03x5               g056(.a(new_n151), .b(new_n148), .c(new_n107), .d(new_n117), .o1(new_n152));
  aoai13aa1n02x5               g057(.a(new_n136), .b(new_n130), .c(new_n120), .d(new_n126), .o1(new_n153));
  nona22aa1n02x4               g058(.a(new_n153), .b(new_n143), .c(new_n135), .out0(new_n154));
  nanp02aa1n02x5               g059(.a(new_n154), .b(new_n144), .o1(new_n155));
  nor022aa1n12x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand42aa1n02x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n152), .c(new_n155), .out0(\s[13] ));
  nanp02aa1n06x5               g064(.a(new_n152), .b(new_n155), .o1(new_n160));
  inv000aa1n02x5               g065(.a(new_n156), .o1(new_n161));
  nor042aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nand02aa1n03x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  oaib12aa1n02x5               g068(.a(new_n161), .b(new_n162), .c(new_n163), .out0(new_n164));
  aoi012aa1n02x5               g069(.a(new_n164), .b(new_n160), .c(new_n158), .o1(new_n165));
  nano23aa1n06x5               g070(.a(new_n156), .b(new_n162), .c(new_n163), .d(new_n157), .out0(new_n166));
  nanp02aa1n02x5               g071(.a(new_n160), .b(new_n166), .o1(new_n167));
  tech160nm_fioaoi03aa1n03p5x5 g072(.a(\a[14] ), .b(\b[13] ), .c(new_n161), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  aoi012aa1n02x5               g074(.a(new_n162), .b(new_n167), .c(new_n169), .o1(new_n170));
  norp02aa1n02x5               g075(.a(new_n170), .b(new_n165), .o1(\s[14] ));
  nor042aa1n04x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nand42aa1n02x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n167), .c(new_n169), .out0(\s[15] ));
  inv000aa1d42x5               g080(.a(new_n172), .o1(new_n176));
  aoai13aa1n06x5               g081(.a(new_n174), .b(new_n168), .c(new_n160), .d(new_n166), .o1(new_n177));
  nor002aa1n02x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nand42aa1n16x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  inv000aa1d42x5               g085(.a(new_n179), .o1(new_n181));
  oai022aa1n02x5               g086(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n182));
  nona22aa1n02x4               g087(.a(new_n177), .b(new_n181), .c(new_n182), .out0(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n180), .c(new_n177), .d(new_n176), .o1(\s[16] ));
  nano23aa1n03x7               g089(.a(new_n172), .b(new_n178), .c(new_n179), .d(new_n173), .out0(new_n185));
  nano22aa1n03x7               g090(.a(new_n150), .b(new_n166), .c(new_n185), .out0(new_n186));
  aoai13aa1n09x5               g091(.a(new_n186), .b(new_n148), .c(new_n107), .d(new_n117), .o1(new_n187));
  nano22aa1n03x5               g092(.a(new_n178), .b(new_n173), .c(new_n179), .out0(new_n188));
  oai012aa1n02x5               g093(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .o1(new_n189));
  oaih12aa1n02x5               g094(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .o1(new_n190));
  nano23aa1n03x7               g095(.a(new_n190), .b(new_n189), .c(new_n161), .d(new_n144), .out0(new_n191));
  oab012aa1n03x5               g096(.a(new_n190), .b(new_n156), .c(new_n162), .out0(new_n192));
  ao0022aa1n03x7               g097(.a(new_n192), .b(new_n188), .c(new_n182), .d(new_n179), .o(new_n193));
  aoi013aa1n06x5               g098(.a(new_n193), .b(new_n154), .c(new_n188), .d(new_n191), .o1(new_n194));
  nanp02aa1n12x5               g099(.a(new_n187), .b(new_n194), .o1(new_n195));
  xorc02aa1n12x5               g100(.a(\a[17] ), .b(\b[16] ), .out0(new_n196));
  aoai13aa1n02x5               g101(.a(new_n185), .b(new_n168), .c(new_n160), .d(new_n166), .o1(new_n197));
  aoi012aa1n02x5               g102(.a(new_n196), .b(new_n179), .c(new_n182), .o1(new_n198));
  aoi022aa1n02x5               g103(.a(new_n197), .b(new_n198), .c(new_n196), .d(new_n195), .o1(\s[17] ));
  norp02aa1n02x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nand02aa1d04x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  obai22aa1n02x7               g106(.a(new_n201), .b(new_n200), .c(\a[17] ), .d(\b[16] ), .out0(new_n202));
  aoi012aa1n02x5               g107(.a(new_n202), .b(new_n195), .c(new_n196), .o1(new_n203));
  nanb03aa1d18x5               g108(.a(new_n200), .b(new_n196), .c(new_n201), .out0(new_n204));
  oai022aa1n02x7               g109(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n205));
  and002aa1n02x5               g110(.a(new_n205), .b(new_n201), .o(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  aoai13aa1n04x5               g112(.a(new_n207), .b(new_n204), .c(new_n187), .d(new_n194), .o1(new_n208));
  aoib12aa1n02x5               g113(.a(new_n203), .b(new_n208), .c(new_n200), .out0(\s[18] ));
  tech160nm_fixorc02aa1n02p5x5 g114(.a(\a[19] ), .b(\b[18] ), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n204), .o1(new_n211));
  aoi112aa1n02x5               g116(.a(new_n210), .b(new_n206), .c(new_n195), .d(new_n211), .o1(new_n212));
  aoi012aa1n02x5               g117(.a(new_n212), .b(new_n208), .c(new_n210), .o1(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g119(.a(\a[19] ), .o1(new_n215));
  inv000aa1d42x5               g120(.a(\b[18] ), .o1(new_n216));
  nanp02aa1n02x5               g121(.a(new_n216), .b(new_n215), .o1(new_n217));
  nanp02aa1n02x5               g122(.a(new_n208), .b(new_n210), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[20] ), .b(\b[19] ), .out0(new_n219));
  nand02aa1d16x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  oai022aa1n02x5               g126(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n222));
  nona22aa1n02x4               g127(.a(new_n218), .b(new_n221), .c(new_n222), .out0(new_n223));
  aoai13aa1n02x5               g128(.a(new_n223), .b(new_n219), .c(new_n217), .d(new_n218), .o1(\s[20] ));
  nano22aa1n12x5               g129(.a(new_n204), .b(new_n210), .c(new_n219), .out0(new_n225));
  inv000aa1d42x5               g130(.a(\a[20] ), .o1(new_n226));
  nanb02aa1n02x5               g131(.a(\b[19] ), .b(new_n226), .out0(new_n227));
  oai112aa1n02x5               g132(.a(new_n227), .b(new_n220), .c(new_n216), .d(new_n215), .o1(new_n228));
  nand23aa1n02x5               g133(.a(new_n205), .b(new_n217), .c(new_n201), .o1(new_n229));
  nanp02aa1n02x5               g134(.a(new_n222), .b(new_n220), .o1(new_n230));
  oai012aa1n06x5               g135(.a(new_n230), .b(new_n229), .c(new_n228), .o1(new_n231));
  xorc02aa1n02x5               g136(.a(\a[21] ), .b(\b[20] ), .out0(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n231), .c(new_n195), .d(new_n225), .o1(new_n233));
  nano32aa1n02x4               g138(.a(new_n228), .b(new_n205), .c(new_n217), .d(new_n201), .out0(new_n234));
  nona22aa1n02x4               g139(.a(new_n230), .b(new_n234), .c(new_n232), .out0(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n195), .c(new_n225), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n233), .b(new_n236), .out0(\s[21] ));
  inv000aa1d42x5               g142(.a(\a[21] ), .o1(new_n238));
  nanb02aa1n02x5               g143(.a(\b[20] ), .b(new_n238), .out0(new_n239));
  xorc02aa1n02x5               g144(.a(\a[22] ), .b(\b[21] ), .out0(new_n240));
  and002aa1n02x5               g145(.a(\b[21] ), .b(\a[22] ), .o(new_n241));
  oai022aa1n02x5               g146(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n242));
  nona22aa1n03x5               g147(.a(new_n233), .b(new_n241), .c(new_n242), .out0(new_n243));
  aoai13aa1n03x5               g148(.a(new_n243), .b(new_n240), .c(new_n239), .d(new_n233), .o1(\s[22] ));
  inv000aa1d42x5               g149(.a(\a[22] ), .o1(new_n245));
  xroi22aa1d04x5               g150(.a(new_n238), .b(\b[20] ), .c(new_n245), .d(\b[21] ), .out0(new_n246));
  nano32aa1n02x4               g151(.a(new_n204), .b(new_n246), .c(new_n210), .d(new_n219), .out0(new_n247));
  oaoi03aa1n02x5               g152(.a(\a[22] ), .b(\b[21] ), .c(new_n239), .o1(new_n248));
  tech160nm_fiao0012aa1n02p5x5 g153(.a(new_n248), .b(new_n231), .c(new_n246), .o(new_n249));
  xorc02aa1n02x5               g154(.a(\a[23] ), .b(\b[22] ), .out0(new_n250));
  aoai13aa1n06x5               g155(.a(new_n250), .b(new_n249), .c(new_n195), .d(new_n247), .o1(new_n251));
  aoi112aa1n02x5               g156(.a(new_n250), .b(new_n248), .c(new_n231), .d(new_n246), .o1(new_n252));
  aobi12aa1n02x5               g157(.a(new_n252), .b(new_n195), .c(new_n247), .out0(new_n253));
  norb02aa1n02x5               g158(.a(new_n251), .b(new_n253), .out0(\s[23] ));
  norp02aa1n02x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[24] ), .b(\b[23] ), .out0(new_n257));
  and002aa1n02x5               g162(.a(\b[23] ), .b(\a[24] ), .o(new_n258));
  oai022aa1n02x5               g163(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n259));
  nona22aa1n03x5               g164(.a(new_n251), .b(new_n258), .c(new_n259), .out0(new_n260));
  aoai13aa1n03x5               g165(.a(new_n260), .b(new_n257), .c(new_n256), .d(new_n251), .o1(\s[24] ));
  inv000aa1d42x5               g166(.a(\a[23] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(\a[24] ), .o1(new_n263));
  xroi22aa1d04x5               g168(.a(new_n262), .b(\b[22] ), .c(new_n263), .d(\b[23] ), .out0(new_n264));
  nanp02aa1n02x5               g169(.a(new_n264), .b(new_n246), .o1(new_n265));
  nano23aa1n02x4               g170(.a(new_n265), .b(new_n204), .c(new_n210), .d(new_n219), .out0(new_n266));
  aoai13aa1n06x5               g171(.a(new_n264), .b(new_n248), .c(new_n231), .d(new_n246), .o1(new_n267));
  oaib12aa1n02x5               g172(.a(new_n259), .b(new_n263), .c(\b[23] ), .out0(new_n268));
  nand22aa1n03x5               g173(.a(new_n267), .b(new_n268), .o1(new_n269));
  tech160nm_fixorc02aa1n03p5x5 g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n269), .c(new_n195), .d(new_n266), .o1(new_n271));
  nanb02aa1n02x5               g176(.a(new_n270), .b(new_n268), .out0(new_n272));
  aoi122aa1n02x7               g177(.a(new_n272), .b(new_n249), .c(new_n264), .d(new_n195), .e(new_n266), .o1(new_n273));
  norb02aa1n02x5               g178(.a(new_n271), .b(new_n273), .out0(\s[25] ));
  norp02aa1n02x5               g179(.a(\b[24] ), .b(\a[25] ), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  xorc02aa1n02x5               g181(.a(\a[26] ), .b(\b[25] ), .out0(new_n277));
  and002aa1n02x5               g182(.a(\b[25] ), .b(\a[26] ), .o(new_n278));
  oai022aa1n02x5               g183(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n279));
  nona22aa1n03x5               g184(.a(new_n271), .b(new_n278), .c(new_n279), .out0(new_n280));
  aoai13aa1n03x5               g185(.a(new_n280), .b(new_n277), .c(new_n276), .d(new_n271), .o1(\s[26] ));
  and002aa1n06x5               g186(.a(new_n277), .b(new_n270), .o(new_n282));
  nanb03aa1n06x5               g187(.a(new_n265), .b(new_n225), .c(new_n282), .out0(new_n283));
  nanb02aa1n09x5               g188(.a(new_n283), .b(new_n195), .out0(new_n284));
  aoi012aa1n06x5               g189(.a(new_n283), .b(new_n187), .c(new_n194), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n282), .o1(new_n286));
  aob012aa1n02x5               g191(.a(new_n279), .b(\b[25] ), .c(\a[26] ), .out0(new_n287));
  aoai13aa1n06x5               g192(.a(new_n287), .b(new_n286), .c(new_n267), .d(new_n268), .o1(new_n288));
  xorc02aa1n12x5               g193(.a(\a[27] ), .b(\b[26] ), .out0(new_n289));
  tech160nm_fioai012aa1n03p5x5 g194(.a(new_n289), .b(new_n288), .c(new_n285), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n278), .o1(new_n291));
  aoi122aa1n02x5               g196(.a(new_n289), .b(new_n291), .c(new_n279), .d(new_n269), .e(new_n282), .o1(new_n292));
  aobi12aa1n03x7               g197(.a(new_n290), .b(new_n292), .c(new_n284), .out0(\s[27] ));
  norp02aa1n02x5               g198(.a(\b[26] ), .b(\a[27] ), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n294), .o1(new_n295));
  xorc02aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .out0(new_n296));
  aoi022aa1n02x5               g201(.a(new_n269), .b(new_n282), .c(new_n291), .d(new_n279), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n289), .o1(new_n298));
  oai022aa1n02x5               g203(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n299));
  aoi012aa1n02x5               g204(.a(new_n299), .b(\a[28] ), .c(\b[27] ), .o1(new_n300));
  aoai13aa1n06x5               g205(.a(new_n300), .b(new_n298), .c(new_n297), .d(new_n284), .o1(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n296), .c(new_n290), .d(new_n295), .o1(\s[28] ));
  xorc02aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .out0(new_n303));
  and002aa1n02x5               g208(.a(new_n296), .b(new_n289), .o(new_n304));
  tech160nm_fioai012aa1n03p5x5 g209(.a(new_n304), .b(new_n288), .c(new_n285), .o1(new_n305));
  inv000aa1d42x5               g210(.a(\b[27] ), .o1(new_n306));
  oaib12aa1n09x5               g211(.a(new_n299), .b(new_n306), .c(\a[28] ), .out0(new_n307));
  nanp03aa1n02x5               g212(.a(new_n305), .b(new_n307), .c(new_n303), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n307), .o1(new_n309));
  oaoi13aa1n03x5               g214(.a(new_n309), .b(new_n304), .c(new_n288), .d(new_n285), .o1(new_n310));
  oai012aa1n02x5               g215(.a(new_n308), .b(new_n310), .c(new_n303), .o1(\s[29] ));
  xorb03aa1n02x5               g216(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n09x5               g217(.a(new_n298), .b(new_n296), .c(new_n303), .out0(new_n313));
  oaoi03aa1n02x5               g218(.a(\a[29] ), .b(\b[28] ), .c(new_n307), .o1(new_n314));
  oaoi13aa1n02x5               g219(.a(new_n314), .b(new_n313), .c(new_n288), .d(new_n285), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .out0(new_n316));
  inv000aa1d42x5               g221(.a(new_n313), .o1(new_n317));
  norb02aa1n02x5               g222(.a(new_n316), .b(new_n314), .out0(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n317), .c(new_n297), .d(new_n284), .o1(new_n319));
  oai012aa1n03x5               g224(.a(new_n319), .b(new_n315), .c(new_n316), .o1(\s[30] ));
  xorc02aa1n02x5               g225(.a(\a[31] ), .b(\b[30] ), .out0(new_n321));
  nano32aa1n09x5               g226(.a(new_n298), .b(new_n316), .c(new_n296), .d(new_n303), .out0(new_n322));
  aoi012aa1n02x5               g227(.a(new_n318), .b(\a[30] ), .c(\b[29] ), .o1(new_n323));
  oaoi13aa1n02x5               g228(.a(new_n323), .b(new_n322), .c(new_n288), .d(new_n285), .o1(new_n324));
  inv000aa1d42x5               g229(.a(new_n322), .o1(new_n325));
  norb02aa1n02x5               g230(.a(new_n321), .b(new_n323), .out0(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n325), .c(new_n297), .d(new_n284), .o1(new_n327));
  oai012aa1n03x5               g232(.a(new_n327), .b(new_n324), .c(new_n321), .o1(\s[31] ));
  nano22aa1n02x4               g233(.a(new_n97), .b(new_n98), .c(new_n99), .out0(new_n329));
  norb02aa1n02x5               g234(.a(new_n104), .b(new_n103), .out0(new_n330));
  nona22aa1n02x4               g235(.a(new_n104), .b(new_n103), .c(new_n97), .out0(new_n331));
  oai022aa1n02x5               g236(.a(new_n331), .b(new_n329), .c(new_n330), .d(new_n100), .o1(\s[3] ));
  nanb02aa1n02x5               g237(.a(new_n101), .b(new_n102), .out0(new_n333));
  oai012aa1n02x5               g238(.a(new_n104), .b(new_n331), .c(new_n329), .o1(new_n334));
  aboi22aa1n03x5               g239(.a(new_n101), .b(new_n107), .c(new_n334), .d(new_n333), .out0(\s[4] ));
  oaoi13aa1n03x5               g240(.a(new_n116), .b(new_n106), .c(new_n105), .d(new_n100), .o1(new_n336));
  oai112aa1n02x5               g241(.a(new_n106), .b(new_n116), .c(new_n105), .d(new_n100), .o1(new_n337));
  norb02aa1n02x5               g242(.a(new_n337), .b(new_n336), .out0(\s[5] ));
  oabi12aa1n02x5               g243(.a(new_n115), .b(new_n336), .c(new_n122), .out0(new_n339));
  inv000aa1d42x5               g244(.a(new_n113), .o1(new_n340));
  aoi112aa1n02x5               g245(.a(new_n336), .b(new_n122), .c(new_n340), .d(new_n114), .o1(new_n341));
  norb02aa1n02x5               g246(.a(new_n339), .b(new_n341), .out0(\s[6] ));
  nanb02aa1n02x5               g247(.a(new_n110), .b(new_n111), .out0(new_n343));
  xobna2aa1n03x5               g248(.a(new_n343), .b(new_n339), .c(new_n340), .out0(\s[7] ));
  norb03aa1n02x5               g249(.a(new_n109), .b(new_n108), .c(new_n110), .out0(new_n345));
  tech160nm_fiaoi012aa1n05x5   g250(.a(new_n343), .b(new_n339), .c(new_n340), .o1(new_n346));
  obai22aa1n02x7               g251(.a(new_n109), .b(new_n108), .c(new_n346), .d(new_n110), .out0(new_n347));
  oaib12aa1n02x5               g252(.a(new_n347), .b(new_n346), .c(new_n345), .out0(\s[8] ));
  nona22aa1n02x4               g253(.a(new_n118), .b(new_n119), .c(new_n124), .out0(new_n349));
  norb02aa1n02x5               g254(.a(new_n127), .b(new_n120), .out0(new_n350));
  norp03aa1n02x5               g255(.a(new_n124), .b(new_n350), .c(new_n119), .o1(new_n351));
  aoi022aa1n02x5               g256(.a(new_n349), .b(new_n350), .c(new_n118), .d(new_n351), .o1(\s[9] ));
endmodule


