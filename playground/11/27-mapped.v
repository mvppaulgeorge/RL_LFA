// Benchmark "adder" written by ABC on Wed Jul 17 17:49:13 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n238, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n342, new_n343, new_n344, new_n347,
    new_n349, new_n350, new_n351, new_n353, new_n354;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n20x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  orn002aa1n02x5               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  aob012aa1n02x5               g005(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(new_n101));
  nor022aa1n06x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand42aa1n03x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanb02aa1n02x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  nor042aa1n04x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor042aa1n06x5               g010(.a(new_n105), .b(new_n102), .o1(new_n106));
  aoai13aa1n06x5               g011(.a(new_n106), .b(new_n104), .c(new_n101), .d(new_n99), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  norp02aa1n04x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  tech160nm_fiaoi012aa1n04x5   g014(.a(new_n109), .b(\a[6] ), .c(\b[5] ), .o1(new_n110));
  inv000aa1d42x5               g015(.a(\a[8] ), .o1(new_n111));
  inv000aa1d42x5               g016(.a(\b[7] ), .o1(new_n112));
  aoi022aa1n02x5               g017(.a(new_n112), .b(new_n111), .c(\a[4] ), .d(\b[3] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nor002aa1n12x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nor022aa1n16x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nona23aa1n03x5               g022(.a(new_n116), .b(new_n114), .c(new_n117), .d(new_n115), .out0(new_n118));
  nano32aa1n02x5               g023(.a(new_n118), .b(new_n113), .c(new_n110), .d(new_n108), .out0(new_n119));
  nanp02aa1n02x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  oaoi13aa1n06x5               g025(.a(new_n115), .b(new_n120), .c(new_n117), .d(new_n109), .o1(new_n121));
  aoi022aa1n02x5               g026(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n122));
  obai22aa1n06x5               g027(.a(new_n122), .b(new_n121), .c(\a[8] ), .d(\b[7] ), .out0(new_n123));
  nanp02aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n97), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n123), .c(new_n119), .d(new_n107), .o1(new_n126));
  nor042aa1n06x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand02aa1n06x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n02x5               g035(.a(new_n101), .b(new_n99), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n103), .b(new_n102), .out0(new_n132));
  inv000aa1d42x5               g037(.a(new_n106), .o1(new_n133));
  aoi012aa1n02x5               g038(.a(new_n133), .b(new_n131), .c(new_n132), .o1(new_n134));
  nano22aa1n02x4               g039(.a(new_n109), .b(new_n108), .c(new_n120), .out0(new_n135));
  nano23aa1n02x4               g040(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n136));
  nano32aa1n02x5               g041(.a(new_n134), .b(new_n136), .c(new_n135), .d(new_n113), .out0(new_n137));
  nona23aa1n06x5               g042(.a(new_n128), .b(new_n124), .c(new_n97), .d(new_n127), .out0(new_n138));
  oabi12aa1n02x5               g043(.a(new_n138), .b(new_n137), .c(new_n123), .out0(new_n139));
  oai012aa1n02x5               g044(.a(new_n128), .b(new_n127), .c(new_n97), .o1(new_n140));
  xnrc02aa1n12x5               g045(.a(\b[10] ), .b(\a[11] ), .out0(new_n141));
  inv000aa1d42x5               g046(.a(new_n141), .o1(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n139), .c(new_n140), .out0(\s[11] ));
  inv000aa1d42x5               g048(.a(\a[11] ), .o1(new_n144));
  inv000aa1d42x5               g049(.a(\b[10] ), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(new_n145), .b(new_n144), .o1(new_n146));
  aob012aa1n02x5               g051(.a(new_n142), .b(new_n139), .c(new_n140), .out0(new_n147));
  inv000aa1d42x5               g052(.a(\a[12] ), .o1(new_n148));
  inv000aa1d42x5               g053(.a(\b[11] ), .o1(new_n149));
  nand42aa1n06x5               g054(.a(new_n149), .b(new_n148), .o1(new_n150));
  nand42aa1n02x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n150), .b(new_n151), .o1(new_n152));
  xobna2aa1n03x5               g057(.a(new_n152), .b(new_n147), .c(new_n146), .out0(\s[12] ));
  norp03aa1n02x5               g058(.a(new_n138), .b(new_n141), .c(new_n152), .o1(new_n154));
  aoai13aa1n02x5               g059(.a(new_n154), .b(new_n123), .c(new_n119), .d(new_n107), .o1(new_n155));
  oai112aa1n04x5               g060(.a(new_n150), .b(new_n151), .c(new_n145), .d(new_n144), .o1(new_n156));
  oai112aa1n06x5               g061(.a(new_n128), .b(new_n146), .c(new_n127), .d(new_n97), .o1(new_n157));
  aoi112aa1n02x5               g062(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n158));
  norb02aa1n03x5               g063(.a(new_n150), .b(new_n158), .out0(new_n159));
  oaih12aa1n12x5               g064(.a(new_n159), .b(new_n157), .c(new_n156), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  nor042aa1n06x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nanb02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n155), .c(new_n161), .out0(\s[13] ));
  inv000aa1d42x5               g071(.a(new_n162), .o1(new_n167));
  nand22aa1n03x5               g072(.a(new_n155), .b(new_n161), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(new_n168), .b(new_n165), .o1(new_n169));
  nor042aa1n02x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  xobna2aa1n03x5               g077(.a(new_n172), .b(new_n169), .c(new_n167), .out0(\s[14] ));
  nano23aa1n06x5               g078(.a(new_n162), .b(new_n170), .c(new_n171), .d(new_n163), .out0(new_n174));
  oaoi03aa1n02x5               g079(.a(\a[14] ), .b(\b[13] ), .c(new_n167), .o1(new_n175));
  nor042aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nanp02aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  norb02aa1n06x4               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  aoai13aa1n06x5               g083(.a(new_n178), .b(new_n175), .c(new_n168), .d(new_n174), .o1(new_n179));
  aoi112aa1n02x5               g084(.a(new_n178), .b(new_n175), .c(new_n168), .d(new_n174), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(\s[15] ));
  nor042aa1n04x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nand02aa1d08x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  norb02aa1n09x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  inv000aa1d42x5               g089(.a(new_n182), .o1(new_n185));
  aoi012aa1n02x5               g090(.a(new_n176), .b(new_n185), .c(new_n183), .o1(new_n186));
  inv000aa1n02x5               g091(.a(new_n176), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(new_n179), .b(new_n187), .o1(new_n188));
  aoi022aa1n02x5               g093(.a(new_n188), .b(new_n184), .c(new_n179), .d(new_n186), .o1(\s[16] ));
  norp02aa1n02x5               g094(.a(new_n141), .b(new_n152), .o1(new_n190));
  nanb02aa1n02x5               g095(.a(new_n138), .b(new_n190), .out0(new_n191));
  nano32aa1n02x5               g096(.a(new_n191), .b(new_n184), .c(new_n174), .d(new_n178), .out0(new_n192));
  tech160nm_fioai012aa1n03p5x5 g097(.a(new_n192), .b(new_n137), .c(new_n123), .o1(new_n193));
  xorc02aa1n02x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  nona23aa1n09x5               g099(.a(new_n171), .b(new_n163), .c(new_n162), .d(new_n170), .out0(new_n195));
  nano22aa1n12x5               g100(.a(new_n195), .b(new_n178), .c(new_n184), .out0(new_n196));
  inv000aa1d42x5               g101(.a(new_n183), .o1(new_n197));
  oai022aa1n02x5               g102(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n198));
  nanp03aa1n02x5               g103(.a(new_n198), .b(new_n171), .c(new_n177), .o1(new_n199));
  aoai13aa1n02x5               g104(.a(new_n185), .b(new_n197), .c(new_n199), .d(new_n187), .o1(new_n200));
  aoi112aa1n02x5               g105(.a(new_n200), .b(new_n194), .c(new_n160), .d(new_n196), .o1(new_n201));
  nand02aa1n03x5               g106(.a(new_n119), .b(new_n107), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n115), .o1(new_n203));
  oaih12aa1n02x5               g108(.a(new_n120), .b(new_n117), .c(new_n109), .o1(new_n204));
  nand42aa1n02x5               g109(.a(new_n204), .b(new_n203), .o1(new_n205));
  aoi022aa1n06x5               g110(.a(new_n205), .b(new_n122), .c(new_n112), .d(new_n111), .o1(new_n206));
  nona32aa1n03x5               g111(.a(new_n196), .b(new_n138), .c(new_n152), .d(new_n141), .out0(new_n207));
  aoi012aa1n12x5               g112(.a(new_n200), .b(new_n160), .c(new_n196), .o1(new_n208));
  aoai13aa1n12x5               g113(.a(new_n208), .b(new_n207), .c(new_n202), .d(new_n206), .o1(new_n209));
  aoi022aa1n02x5               g114(.a(new_n209), .b(new_n194), .c(new_n193), .d(new_n201), .o1(\s[17] ));
  inv000aa1d42x5               g115(.a(\a[17] ), .o1(new_n211));
  inv030aa1d32x5               g116(.a(\b[16] ), .o1(new_n212));
  nanp02aa1n02x5               g117(.a(new_n212), .b(new_n211), .o1(new_n213));
  oaib12aa1n02x5               g118(.a(new_n209), .b(new_n212), .c(\a[17] ), .out0(new_n214));
  nor002aa1d32x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  nand42aa1n16x5               g120(.a(\b[17] ), .b(\a[18] ), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  xnbna2aa1n03x5               g122(.a(new_n217), .b(new_n214), .c(new_n213), .out0(\s[18] ));
  nanp02aa1n02x5               g123(.a(\b[16] ), .b(\a[17] ), .o1(new_n219));
  nano32aa1n03x7               g124(.a(new_n215), .b(new_n213), .c(new_n216), .d(new_n219), .out0(new_n220));
  aoai13aa1n12x5               g125(.a(new_n216), .b(new_n215), .c(new_n211), .d(new_n212), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  tech160nm_fixorc02aa1n04x5   g127(.a(\a[19] ), .b(\b[18] ), .out0(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n222), .c(new_n209), .d(new_n220), .o1(new_n224));
  aoi112aa1n02x5               g129(.a(new_n223), .b(new_n222), .c(new_n209), .d(new_n220), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n224), .b(new_n225), .out0(\s[19] ));
  xnrc02aa1n02x5               g131(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  xorc02aa1n02x5               g132(.a(\a[20] ), .b(\b[19] ), .out0(new_n228));
  inv000aa1d42x5               g133(.a(\a[19] ), .o1(new_n229));
  inv000aa1d42x5               g134(.a(\b[18] ), .o1(new_n230));
  inv000aa1d42x5               g135(.a(\a[20] ), .o1(new_n231));
  inv000aa1d42x5               g136(.a(\b[19] ), .o1(new_n232));
  nanp02aa1n02x5               g137(.a(new_n232), .b(new_n231), .o1(new_n233));
  nanp02aa1n02x5               g138(.a(\b[19] ), .b(\a[20] ), .o1(new_n234));
  aoi022aa1n02x5               g139(.a(new_n233), .b(new_n234), .c(new_n230), .d(new_n229), .o1(new_n235));
  nor002aa1n02x5               g140(.a(\b[18] ), .b(\a[19] ), .o1(new_n236));
  inv000aa1n02x5               g141(.a(new_n236), .o1(new_n237));
  nanp02aa1n02x5               g142(.a(new_n224), .b(new_n237), .o1(new_n238));
  aoi022aa1n02x5               g143(.a(new_n238), .b(new_n228), .c(new_n224), .d(new_n235), .o1(\s[20] ));
  nand23aa1n06x5               g144(.a(new_n220), .b(new_n223), .c(new_n228), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  oai112aa1n02x5               g146(.a(new_n233), .b(new_n234), .c(new_n230), .d(new_n229), .o1(new_n242));
  oaoi03aa1n03x5               g147(.a(new_n231), .b(new_n232), .c(new_n236), .o1(new_n243));
  oai013aa1n03x5               g148(.a(new_n243), .b(new_n242), .c(new_n221), .d(new_n236), .o1(new_n244));
  xorc02aa1n12x5               g149(.a(\a[21] ), .b(\b[20] ), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n244), .c(new_n209), .d(new_n241), .o1(new_n246));
  aoi112aa1n02x5               g151(.a(new_n245), .b(new_n244), .c(new_n209), .d(new_n241), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n246), .b(new_n247), .out0(\s[21] ));
  xorc02aa1n12x5               g153(.a(\a[22] ), .b(\b[21] ), .out0(new_n249));
  nor042aa1n03x5               g154(.a(\b[20] ), .b(\a[21] ), .o1(new_n250));
  norp02aa1n02x5               g155(.a(new_n249), .b(new_n250), .o1(new_n251));
  inv000aa1n03x5               g156(.a(new_n250), .o1(new_n252));
  nanp02aa1n02x5               g157(.a(new_n246), .b(new_n252), .o1(new_n253));
  aoi022aa1n02x5               g158(.a(new_n253), .b(new_n249), .c(new_n246), .d(new_n251), .o1(\s[22] ));
  nand02aa1d06x5               g159(.a(new_n249), .b(new_n245), .o1(new_n255));
  nona22aa1n06x5               g160(.a(new_n209), .b(new_n240), .c(new_n255), .out0(new_n256));
  nona22aa1n09x5               g161(.a(new_n237), .b(new_n242), .c(new_n221), .out0(new_n257));
  oaoi03aa1n12x5               g162(.a(\a[22] ), .b(\b[21] ), .c(new_n252), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  aoai13aa1n12x5               g164(.a(new_n259), .b(new_n255), .c(new_n257), .d(new_n243), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  xnrc02aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .out0(new_n262));
  xobna2aa1n03x5               g167(.a(new_n262), .b(new_n256), .c(new_n261), .out0(\s[23] ));
  tech160nm_fiao0012aa1n02p5x5 g168(.a(new_n262), .b(new_n256), .c(new_n261), .o(new_n264));
  xorc02aa1n02x5               g169(.a(\a[24] ), .b(\b[23] ), .out0(new_n265));
  nor042aa1n03x5               g170(.a(\b[22] ), .b(\a[23] ), .o1(new_n266));
  norp02aa1n02x5               g171(.a(new_n265), .b(new_n266), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n266), .o1(new_n268));
  aoai13aa1n03x5               g173(.a(new_n268), .b(new_n262), .c(new_n256), .d(new_n261), .o1(new_n269));
  aoi022aa1n03x5               g174(.a(new_n264), .b(new_n267), .c(new_n269), .d(new_n265), .o1(\s[24] ));
  inv000aa1n02x5               g175(.a(new_n255), .o1(new_n271));
  norb02aa1n09x5               g176(.a(new_n265), .b(new_n262), .out0(new_n272));
  nano22aa1n02x4               g177(.a(new_n240), .b(new_n271), .c(new_n272), .out0(new_n273));
  nanp02aa1n06x5               g178(.a(new_n209), .b(new_n273), .o1(new_n274));
  oao003aa1n02x5               g179(.a(\a[24] ), .b(\b[23] ), .c(new_n268), .carry(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  tech160nm_fiaoi012aa1n02p5x5 g181(.a(new_n276), .b(new_n260), .c(new_n272), .o1(new_n277));
  nor042aa1n06x5               g182(.a(\b[24] ), .b(\a[25] ), .o1(new_n278));
  and002aa1n02x5               g183(.a(\b[24] ), .b(\a[25] ), .o(new_n279));
  nor042aa1n03x5               g184(.a(new_n279), .b(new_n278), .o1(new_n280));
  aob012aa1n03x5               g185(.a(new_n280), .b(new_n274), .c(new_n277), .out0(new_n281));
  aoi112aa1n02x5               g186(.a(new_n280), .b(new_n276), .c(new_n260), .d(new_n272), .o1(new_n282));
  aobi12aa1n02x7               g187(.a(new_n281), .b(new_n282), .c(new_n274), .out0(\s[25] ));
  xorc02aa1n02x5               g188(.a(\a[26] ), .b(\b[25] ), .out0(new_n284));
  norp02aa1n02x5               g189(.a(new_n284), .b(new_n278), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n278), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n286), .b(new_n279), .c(new_n274), .d(new_n277), .o1(new_n287));
  aoi022aa1n02x7               g192(.a(new_n287), .b(new_n284), .c(new_n281), .d(new_n285), .o1(\s[26] ));
  and002aa1n02x5               g193(.a(new_n284), .b(new_n280), .o(new_n289));
  aoai13aa1n09x5               g194(.a(new_n289), .b(new_n276), .c(new_n260), .d(new_n272), .o1(new_n290));
  norp02aa1n02x5               g195(.a(\b[26] ), .b(\a[27] ), .o1(new_n291));
  and002aa1n02x5               g196(.a(\b[26] ), .b(\a[27] ), .o(new_n292));
  norp02aa1n02x5               g197(.a(new_n292), .b(new_n291), .o1(new_n293));
  nano32aa1n02x5               g198(.a(new_n240), .b(new_n289), .c(new_n271), .d(new_n272), .out0(new_n294));
  oao003aa1n02x5               g199(.a(\a[26] ), .b(\b[25] ), .c(new_n286), .carry(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  aoi112aa1n02x5               g201(.a(new_n296), .b(new_n293), .c(new_n209), .d(new_n294), .o1(new_n297));
  aoi012aa1n09x5               g202(.a(new_n296), .b(new_n209), .c(new_n294), .o1(new_n298));
  nanp02aa1n02x5               g203(.a(new_n298), .b(new_n290), .o1(new_n299));
  aoi022aa1n02x5               g204(.a(new_n299), .b(new_n293), .c(new_n290), .d(new_n297), .o1(\s[27] ));
  aoai13aa1n03x5               g205(.a(new_n272), .b(new_n258), .c(new_n244), .d(new_n271), .o1(new_n301));
  aobi12aa1n02x5               g206(.a(new_n289), .b(new_n301), .c(new_n275), .out0(new_n302));
  nanp02aa1n02x5               g207(.a(\b[26] ), .b(\a[27] ), .o1(new_n303));
  nona23aa1n03x5               g208(.a(new_n289), .b(new_n272), .c(new_n240), .d(new_n255), .out0(new_n304));
  aoai13aa1n02x7               g209(.a(new_n295), .b(new_n304), .c(new_n193), .d(new_n208), .o1(new_n305));
  oai012aa1n02x5               g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  inv000aa1n03x5               g211(.a(new_n291), .o1(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n292), .c(new_n298), .d(new_n290), .o1(new_n308));
  xorc02aa1n02x5               g213(.a(\a[28] ), .b(\b[27] ), .out0(new_n309));
  norp02aa1n02x5               g214(.a(new_n309), .b(new_n291), .o1(new_n310));
  aoi022aa1n03x5               g215(.a(new_n308), .b(new_n309), .c(new_n306), .d(new_n310), .o1(\s[28] ));
  inv000aa1d42x5               g216(.a(\a[27] ), .o1(new_n312));
  inv000aa1d42x5               g217(.a(\a[28] ), .o1(new_n313));
  xroi22aa1d06x4               g218(.a(new_n312), .b(\b[26] ), .c(new_n313), .d(\b[27] ), .out0(new_n314));
  oai012aa1n02x5               g219(.a(new_n314), .b(new_n305), .c(new_n302), .o1(new_n315));
  inv000aa1n06x5               g220(.a(new_n314), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[28] ), .b(\b[27] ), .c(new_n307), .carry(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n316), .c(new_n298), .d(new_n290), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[29] ), .b(\b[28] ), .out0(new_n319));
  norb02aa1n02x5               g224(.a(new_n317), .b(new_n319), .out0(new_n320));
  aoi022aa1n03x5               g225(.a(new_n318), .b(new_n319), .c(new_n315), .d(new_n320), .o1(\s[29] ));
  xorb03aa1n02x5               g226(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g227(.a(new_n309), .b(new_n319), .c(new_n293), .o(new_n323));
  oai012aa1n02x5               g228(.a(new_n323), .b(new_n305), .c(new_n302), .o1(new_n324));
  inv000aa1d42x5               g229(.a(new_n323), .o1(new_n325));
  oao003aa1n02x5               g230(.a(\a[29] ), .b(\b[28] ), .c(new_n317), .carry(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n325), .c(new_n298), .d(new_n290), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .out0(new_n328));
  norb02aa1n02x5               g233(.a(new_n326), .b(new_n328), .out0(new_n329));
  aoi022aa1n03x5               g234(.a(new_n327), .b(new_n328), .c(new_n324), .d(new_n329), .o1(\s[30] ));
  nano22aa1n02x4               g235(.a(new_n316), .b(new_n319), .c(new_n328), .out0(new_n331));
  oai012aa1n02x5               g236(.a(new_n331), .b(new_n305), .c(new_n302), .o1(new_n332));
  xorc02aa1n02x5               g237(.a(\a[31] ), .b(\b[30] ), .out0(new_n333));
  and002aa1n02x5               g238(.a(\b[29] ), .b(\a[30] ), .o(new_n334));
  oabi12aa1n02x5               g239(.a(new_n333), .b(\a[30] ), .c(\b[29] ), .out0(new_n335));
  oab012aa1n02x4               g240(.a(new_n335), .b(new_n326), .c(new_n334), .out0(new_n336));
  inv000aa1n02x5               g241(.a(new_n331), .o1(new_n337));
  oao003aa1n02x5               g242(.a(\a[30] ), .b(\b[29] ), .c(new_n326), .carry(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n337), .c(new_n298), .d(new_n290), .o1(new_n339));
  aoi022aa1n03x5               g244(.a(new_n339), .b(new_n333), .c(new_n332), .d(new_n336), .o1(\s[31] ));
  xnbna2aa1n03x5               g245(.a(new_n132), .b(new_n101), .c(new_n99), .out0(\s[3] ));
  xorc02aa1n02x5               g246(.a(\a[4] ), .b(\b[3] ), .out0(new_n342));
  aoi112aa1n02x5               g247(.a(new_n102), .b(new_n342), .c(new_n131), .d(new_n132), .o1(new_n343));
  aob012aa1n02x5               g248(.a(new_n107), .b(\b[3] ), .c(\a[4] ), .out0(new_n344));
  oab012aa1n02x4               g249(.a(new_n343), .b(new_n344), .c(new_n105), .out0(\s[4] ));
  xnrb03aa1n02x5               g250(.a(new_n344), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fioaoi03aa1n03p5x5 g251(.a(\a[5] ), .b(\b[4] ), .c(new_n344), .o1(new_n347));
  xorb03aa1n02x5               g252(.a(new_n347), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g253(.a(new_n117), .o1(new_n349));
  nanp03aa1n02x5               g254(.a(new_n347), .b(new_n120), .c(new_n349), .o1(new_n350));
  norb02aa1n02x5               g255(.a(new_n116), .b(new_n115), .out0(new_n351));
  xnbna2aa1n03x5               g256(.a(new_n351), .b(new_n350), .c(new_n349), .out0(\s[7] ));
  aoai13aa1n02x5               g257(.a(new_n351), .b(new_n117), .c(new_n347), .d(new_n120), .o1(new_n353));
  xorc02aa1n02x5               g258(.a(\a[8] ), .b(\b[7] ), .out0(new_n354));
  xnbna2aa1n03x5               g259(.a(new_n354), .b(new_n353), .c(new_n203), .out0(\s[8] ));
  xnbna2aa1n03x5               g260(.a(new_n125), .b(new_n202), .c(new_n206), .out0(\s[9] ));
endmodule

