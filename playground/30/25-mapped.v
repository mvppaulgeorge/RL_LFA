// Benchmark "adder" written by ABC on Thu Jul 18 03:32:50 2024

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
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n168, new_n169, new_n170,
    new_n171, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n335, new_n336, new_n338,
    new_n339, new_n341, new_n342, new_n343, new_n344, new_n345, new_n347,
    new_n348, new_n349, new_n351, new_n352, new_n354, new_n355, new_n356,
    new_n357;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n08x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nanp02aa1n09x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nand42aa1d28x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nor022aa1n16x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nanb03aa1n12x5               g006(.a(new_n101), .b(new_n99), .c(new_n100), .out0(new_n102));
  nand22aa1n06x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nor022aa1n08x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  norb03aa1n03x5               g009(.a(new_n100), .b(new_n104), .c(new_n103), .out0(new_n105));
  nor042aa1n04x5               g010(.a(new_n105), .b(new_n102), .o1(new_n106));
  oai022aa1d24x5               g011(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n107));
  tech160nm_finand02aa1n05x5   g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nand42aa1n06x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nanb03aa1n06x5               g015(.a(new_n110), .b(new_n108), .c(new_n109), .out0(new_n111));
  inv000aa1d42x5               g016(.a(\a[4] ), .o1(new_n112));
  inv000aa1d42x5               g017(.a(\b[3] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\b[7] ), .o1(new_n114));
  nanb02aa1n12x5               g019(.a(\a[8] ), .b(new_n114), .out0(new_n115));
  nanp02aa1n09x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  oai112aa1n02x5               g021(.a(new_n115), .b(new_n116), .c(new_n113), .d(new_n112), .o1(new_n117));
  inv040aa1d32x5               g022(.a(\a[7] ), .o1(new_n118));
  inv040aa1n16x5               g023(.a(\b[6] ), .o1(new_n119));
  nanp02aa1n04x5               g024(.a(new_n119), .b(new_n118), .o1(new_n120));
  nanp02aa1n04x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  oai112aa1n02x5               g026(.a(new_n120), .b(new_n121), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  nor043aa1n03x5               g027(.a(new_n117), .b(new_n122), .c(new_n111), .o1(new_n123));
  oai012aa1n12x5               g028(.a(new_n123), .b(new_n106), .c(new_n107), .o1(new_n124));
  oai022aa1d18x5               g029(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n125));
  and003aa1n03x7               g030(.a(new_n109), .b(new_n121), .c(new_n116), .o(new_n126));
  nor002aa1d32x5               g031(.a(\b[5] ), .b(\a[6] ), .o1(new_n127));
  oab012aa1n09x5               g032(.a(new_n125), .b(new_n110), .c(new_n127), .out0(new_n128));
  aoi022aa1n09x5               g033(.a(new_n128), .b(new_n126), .c(new_n116), .d(new_n125), .o1(new_n129));
  nanp02aa1n03x5               g034(.a(new_n124), .b(new_n129), .o1(new_n130));
  xnrc02aa1n12x5               g035(.a(\b[8] ), .b(\a[9] ), .out0(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(new_n130), .b(new_n132), .o1(new_n133));
  tech160nm_fixnrc02aa1n05x5   g038(.a(\b[9] ), .b(\a[10] ), .out0(new_n134));
  inv030aa1n02x5               g039(.a(new_n134), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n133), .c(new_n98), .out0(\s[10] ));
  aoai13aa1n06x5               g041(.a(new_n135), .b(new_n97), .c(new_n130), .d(new_n132), .o1(new_n137));
  nanp02aa1n04x5               g042(.a(\b[9] ), .b(\a[10] ), .o1(new_n138));
  oai022aa1d24x5               g043(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(new_n139), .b(new_n138), .o1(new_n140));
  nor002aa1d32x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand02aa1n06x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norb02aa1n12x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n137), .c(new_n140), .out0(\s[11] ));
  aob012aa1n02x5               g049(.a(new_n143), .b(new_n137), .c(new_n140), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n141), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n143), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n146), .b(new_n147), .c(new_n137), .d(new_n140), .o1(new_n148));
  nor022aa1n16x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nand22aa1n12x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nanb02aa1n02x5               g055(.a(new_n149), .b(new_n150), .out0(new_n151));
  inv030aa1n02x5               g056(.a(new_n149), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n141), .b(new_n152), .c(new_n150), .o1(new_n153));
  aboi22aa1n03x5               g058(.a(new_n151), .b(new_n148), .c(new_n145), .d(new_n153), .out0(\s[12] ));
  nona23aa1n09x5               g059(.a(new_n135), .b(new_n143), .c(new_n131), .d(new_n151), .out0(new_n155));
  nanb03aa1n09x5               g060(.a(new_n149), .b(new_n150), .c(new_n142), .out0(new_n156));
  oai112aa1n06x5               g061(.a(new_n139), .b(new_n138), .c(\b[10] ), .d(\a[11] ), .o1(new_n157));
  aob012aa1n03x5               g062(.a(new_n152), .b(new_n141), .c(new_n150), .out0(new_n158));
  oabi12aa1n18x5               g063(.a(new_n158), .b(new_n157), .c(new_n156), .out0(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n06x5               g065(.a(new_n160), .b(new_n155), .c(new_n124), .d(new_n129), .o1(new_n161));
  xnrc02aa1n12x5               g066(.a(\b[12] ), .b(\a[13] ), .out0(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  nona23aa1n02x4               g068(.a(new_n150), .b(new_n142), .c(new_n141), .d(new_n149), .out0(new_n164));
  nor043aa1n02x5               g069(.a(new_n164), .b(new_n134), .c(new_n131), .o1(new_n165));
  aoi112aa1n02x5               g070(.a(new_n163), .b(new_n159), .c(new_n130), .d(new_n165), .o1(new_n166));
  aoi012aa1n02x5               g071(.a(new_n166), .b(new_n161), .c(new_n163), .o1(\s[13] ));
  nor042aa1n06x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(new_n161), .b(new_n163), .o1(new_n170));
  tech160nm_fixnrc02aa1n05x5   g075(.a(\b[13] ), .b(\a[14] ), .out0(new_n171));
  xobna2aa1n03x5               g076(.a(new_n171), .b(new_n170), .c(new_n169), .out0(\s[14] ));
  nor042aa1n02x5               g077(.a(new_n171), .b(new_n162), .o1(new_n173));
  oaoi03aa1n09x5               g078(.a(\a[14] ), .b(\b[13] ), .c(new_n169), .o1(new_n174));
  norp02aa1n24x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nanp02aa1n04x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n174), .c(new_n161), .d(new_n173), .o1(new_n178));
  aoi112aa1n02x5               g083(.a(new_n177), .b(new_n174), .c(new_n161), .d(new_n173), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n178), .b(new_n179), .out0(\s[15] ));
  oaih12aa1n02x5               g085(.a(new_n178), .b(\b[14] ), .c(\a[15] ), .o1(new_n181));
  nor002aa1d32x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nand02aa1n06x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  norb02aa1n02x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  aoib12aa1n02x5               g089(.a(new_n175), .b(new_n183), .c(new_n182), .out0(new_n185));
  aoi022aa1n02x5               g090(.a(new_n181), .b(new_n184), .c(new_n178), .d(new_n185), .o1(\s[16] ));
  nona23aa1d16x5               g091(.a(new_n183), .b(new_n176), .c(new_n175), .d(new_n182), .out0(new_n187));
  inv000aa1n02x5               g092(.a(new_n187), .o1(new_n188));
  aoai13aa1n12x5               g093(.a(new_n188), .b(new_n174), .c(new_n159), .d(new_n173), .o1(new_n189));
  inv040aa1n08x5               g094(.a(new_n189), .o1(new_n190));
  nona32aa1n02x5               g095(.a(new_n165), .b(new_n187), .c(new_n171), .d(new_n162), .out0(new_n191));
  aoi012aa1n06x5               g096(.a(new_n182), .b(new_n175), .c(new_n183), .o1(new_n192));
  aoai13aa1n12x5               g097(.a(new_n192), .b(new_n191), .c(new_n124), .d(new_n129), .o1(new_n193));
  xorc02aa1n12x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  oai012aa1n06x5               g099(.a(new_n194), .b(new_n193), .c(new_n190), .o1(new_n195));
  nano22aa1n06x5               g100(.a(new_n155), .b(new_n173), .c(new_n188), .out0(new_n196));
  nanp02aa1n04x5               g101(.a(new_n130), .b(new_n196), .o1(new_n197));
  nano32aa1n02x4               g102(.a(new_n194), .b(new_n197), .c(new_n189), .d(new_n192), .out0(new_n198));
  norb02aa1n02x7               g103(.a(new_n195), .b(new_n198), .out0(\s[17] ));
  inv000aa1d42x5               g104(.a(\a[17] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(\b[16] ), .b(new_n200), .out0(new_n201));
  xorc02aa1n02x5               g106(.a(\a[18] ), .b(\b[17] ), .out0(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n195), .c(new_n201), .out0(\s[18] ));
  inv040aa1d32x5               g108(.a(\a[18] ), .o1(new_n204));
  xroi22aa1d04x5               g109(.a(new_n200), .b(\b[16] ), .c(new_n204), .d(\b[17] ), .out0(new_n205));
  oai012aa1n06x5               g110(.a(new_n205), .b(new_n193), .c(new_n190), .o1(new_n206));
  tech160nm_fioaoi03aa1n03p5x5 g111(.a(\a[18] ), .b(\b[17] ), .c(new_n201), .o1(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  nor002aa1d32x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nand02aa1n08x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  norb02aa1d21x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n206), .c(new_n208), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp03aa1d12x5               g118(.a(new_n197), .b(new_n189), .c(new_n192), .o1(new_n214));
  aoai13aa1n03x5               g119(.a(new_n211), .b(new_n207), .c(new_n214), .d(new_n205), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n209), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n211), .o1(new_n217));
  aoai13aa1n02x7               g122(.a(new_n216), .b(new_n217), .c(new_n206), .d(new_n208), .o1(new_n218));
  nor002aa1n20x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nand22aa1n12x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  norb02aa1n06x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  inv000aa1d42x5               g126(.a(\a[19] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[18] ), .o1(new_n223));
  aboi22aa1n03x5               g128(.a(new_n219), .b(new_n220), .c(new_n222), .d(new_n223), .out0(new_n224));
  aoi022aa1n03x5               g129(.a(new_n218), .b(new_n221), .c(new_n215), .d(new_n224), .o1(\s[20] ));
  nano32aa1n03x7               g130(.a(new_n217), .b(new_n202), .c(new_n194), .d(new_n221), .out0(new_n226));
  oai012aa1n06x5               g131(.a(new_n226), .b(new_n193), .c(new_n190), .o1(new_n227));
  nanb03aa1n12x5               g132(.a(new_n219), .b(new_n220), .c(new_n210), .out0(new_n228));
  inv040aa1d32x5               g133(.a(\b[17] ), .o1(new_n229));
  oai022aa1d24x5               g134(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n230));
  oai122aa1n12x5               g135(.a(new_n230), .b(\a[19] ), .c(\b[18] ), .d(new_n204), .e(new_n229), .o1(new_n231));
  aoi012aa1n06x5               g136(.a(new_n219), .b(new_n209), .c(new_n220), .o1(new_n232));
  oai012aa1d24x5               g137(.a(new_n232), .b(new_n231), .c(new_n228), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  xnrc02aa1n12x5               g139(.a(\b[20] ), .b(\a[21] ), .out0(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  xnbna2aa1n03x5               g141(.a(new_n236), .b(new_n227), .c(new_n234), .out0(\s[21] ));
  aoai13aa1n03x5               g142(.a(new_n236), .b(new_n233), .c(new_n214), .d(new_n226), .o1(new_n238));
  nor042aa1d18x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  inv000aa1n03x5               g144(.a(new_n239), .o1(new_n240));
  aoai13aa1n02x7               g145(.a(new_n240), .b(new_n235), .c(new_n227), .d(new_n234), .o1(new_n241));
  xnrc02aa1n12x5               g146(.a(\b[21] ), .b(\a[22] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n242), .b(new_n239), .out0(new_n244));
  aoi022aa1n03x5               g149(.a(new_n241), .b(new_n243), .c(new_n238), .d(new_n244), .o1(\s[22] ));
  nor042aa1n06x5               g150(.a(new_n242), .b(new_n235), .o1(new_n246));
  and002aa1n02x5               g151(.a(new_n226), .b(new_n246), .o(new_n247));
  oao003aa1n12x5               g152(.a(\a[22] ), .b(\b[21] ), .c(new_n240), .carry(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  aoi012aa1n02x5               g154(.a(new_n249), .b(new_n233), .c(new_n246), .o1(new_n250));
  inv000aa1n02x5               g155(.a(new_n250), .o1(new_n251));
  xorc02aa1n12x5               g156(.a(\a[23] ), .b(\b[22] ), .out0(new_n252));
  aoai13aa1n03x5               g157(.a(new_n252), .b(new_n251), .c(new_n214), .d(new_n247), .o1(new_n253));
  tech160nm_fioai012aa1n04x5   g158(.a(new_n247), .b(new_n193), .c(new_n190), .o1(new_n254));
  nano22aa1n02x4               g159(.a(new_n219), .b(new_n210), .c(new_n220), .out0(new_n255));
  oai022aa1n02x5               g160(.a(new_n204), .b(new_n229), .c(\b[18] ), .d(\a[19] ), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n230), .b(new_n256), .out0(new_n257));
  inv000aa1n02x5               g162(.a(new_n232), .o1(new_n258));
  aoai13aa1n06x5               g163(.a(new_n246), .b(new_n258), .c(new_n257), .d(new_n255), .o1(new_n259));
  nano32aa1n03x5               g164(.a(new_n252), .b(new_n254), .c(new_n259), .d(new_n248), .out0(new_n260));
  norb02aa1n02x7               g165(.a(new_n253), .b(new_n260), .out0(\s[23] ));
  nor042aa1n09x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n252), .o1(new_n264));
  aoai13aa1n04x5               g169(.a(new_n263), .b(new_n264), .c(new_n254), .d(new_n250), .o1(new_n265));
  tech160nm_fixorc02aa1n02p5x5 g170(.a(\a[24] ), .b(\b[23] ), .out0(new_n266));
  norp02aa1n02x5               g171(.a(new_n266), .b(new_n262), .o1(new_n267));
  aoi022aa1n02x7               g172(.a(new_n265), .b(new_n266), .c(new_n253), .d(new_n267), .o1(\s[24] ));
  inv000aa1n02x5               g173(.a(new_n226), .o1(new_n269));
  and002aa1n02x5               g174(.a(new_n266), .b(new_n252), .o(new_n270));
  nano22aa1n02x5               g175(.a(new_n269), .b(new_n270), .c(new_n246), .out0(new_n271));
  oaih12aa1n02x5               g176(.a(new_n271), .b(new_n193), .c(new_n190), .o1(new_n272));
  inv000aa1n02x5               g177(.a(new_n270), .o1(new_n273));
  oao003aa1n02x5               g178(.a(\a[24] ), .b(\b[23] ), .c(new_n263), .carry(new_n274));
  aoai13aa1n04x5               g179(.a(new_n274), .b(new_n273), .c(new_n259), .d(new_n248), .o1(new_n275));
  xorc02aa1n12x5               g180(.a(\a[25] ), .b(\b[24] ), .out0(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n275), .c(new_n214), .d(new_n271), .o1(new_n277));
  aoai13aa1n04x5               g182(.a(new_n270), .b(new_n249), .c(new_n233), .d(new_n246), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n276), .o1(new_n279));
  and003aa1n02x5               g184(.a(new_n278), .b(new_n279), .c(new_n274), .o(new_n280));
  aobi12aa1n02x7               g185(.a(new_n277), .b(new_n280), .c(new_n272), .out0(\s[25] ));
  inv000aa1n02x5               g186(.a(new_n275), .o1(new_n282));
  nor042aa1n03x5               g187(.a(\b[24] ), .b(\a[25] ), .o1(new_n283));
  inv040aa1n03x5               g188(.a(new_n283), .o1(new_n284));
  aoai13aa1n02x7               g189(.a(new_n284), .b(new_n279), .c(new_n272), .d(new_n282), .o1(new_n285));
  tech160nm_fixorc02aa1n03p5x5 g190(.a(\a[26] ), .b(\b[25] ), .out0(new_n286));
  norp02aa1n02x5               g191(.a(new_n286), .b(new_n283), .o1(new_n287));
  aoi022aa1n02x7               g192(.a(new_n285), .b(new_n286), .c(new_n277), .d(new_n287), .o1(\s[26] ));
  and002aa1n02x5               g193(.a(new_n286), .b(new_n276), .o(new_n289));
  nano32aa1n09x5               g194(.a(new_n269), .b(new_n289), .c(new_n246), .d(new_n270), .out0(new_n290));
  oai012aa1n06x5               g195(.a(new_n290), .b(new_n193), .c(new_n190), .o1(new_n291));
  inv000aa1n02x5               g196(.a(new_n289), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[26] ), .b(\b[25] ), .c(new_n284), .carry(new_n293));
  aoai13aa1n04x5               g198(.a(new_n293), .b(new_n292), .c(new_n278), .d(new_n274), .o1(new_n294));
  xorc02aa1n12x5               g199(.a(\a[27] ), .b(\b[26] ), .out0(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n294), .c(new_n214), .d(new_n290), .o1(new_n296));
  inv000aa1n02x5               g201(.a(new_n293), .o1(new_n297));
  aoi112aa1n02x5               g202(.a(new_n295), .b(new_n297), .c(new_n275), .d(new_n289), .o1(new_n298));
  aobi12aa1n02x7               g203(.a(new_n296), .b(new_n298), .c(new_n291), .out0(\s[27] ));
  tech160nm_fiaoi012aa1n05x5   g204(.a(new_n297), .b(new_n275), .c(new_n289), .o1(new_n300));
  norp02aa1n02x5               g205(.a(\b[26] ), .b(\a[27] ), .o1(new_n301));
  inv000aa1n03x5               g206(.a(new_n301), .o1(new_n302));
  inv000aa1n02x5               g207(.a(new_n295), .o1(new_n303));
  aoai13aa1n02x7               g208(.a(new_n302), .b(new_n303), .c(new_n291), .d(new_n300), .o1(new_n304));
  tech160nm_fixorc02aa1n03p5x5 g209(.a(\a[28] ), .b(\b[27] ), .out0(new_n305));
  norp02aa1n02x5               g210(.a(new_n305), .b(new_n301), .o1(new_n306));
  aoi022aa1n03x5               g211(.a(new_n304), .b(new_n305), .c(new_n296), .d(new_n306), .o1(\s[28] ));
  and002aa1n02x5               g212(.a(new_n305), .b(new_n295), .o(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n294), .c(new_n214), .d(new_n290), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n308), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .c(new_n302), .carry(new_n311));
  aoai13aa1n02x7               g216(.a(new_n311), .b(new_n310), .c(new_n291), .d(new_n300), .o1(new_n312));
  tech160nm_fixorc02aa1n03p5x5 g217(.a(\a[29] ), .b(\b[28] ), .out0(new_n313));
  norb02aa1n02x5               g218(.a(new_n311), .b(new_n313), .out0(new_n314));
  aoi022aa1n03x5               g219(.a(new_n312), .b(new_n313), .c(new_n309), .d(new_n314), .o1(\s[29] ));
  xorb03aa1n02x5               g220(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g221(.a(new_n303), .b(new_n305), .c(new_n313), .out0(new_n317));
  aoai13aa1n06x5               g222(.a(new_n317), .b(new_n294), .c(new_n214), .d(new_n290), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n317), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .c(new_n311), .carry(new_n320));
  aoai13aa1n02x7               g225(.a(new_n320), .b(new_n319), .c(new_n291), .d(new_n300), .o1(new_n321));
  xorc02aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .out0(new_n322));
  norb02aa1n02x5               g227(.a(new_n320), .b(new_n322), .out0(new_n323));
  aoi022aa1n03x5               g228(.a(new_n321), .b(new_n322), .c(new_n318), .d(new_n323), .o1(\s[30] ));
  xorc02aa1n02x5               g229(.a(\a[31] ), .b(\b[30] ), .out0(new_n325));
  nano32aa1n03x7               g230(.a(new_n303), .b(new_n322), .c(new_n305), .d(new_n313), .out0(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n294), .c(new_n214), .d(new_n290), .o1(new_n327));
  inv000aa1d42x5               g232(.a(new_n326), .o1(new_n328));
  oao003aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .c(new_n320), .carry(new_n329));
  aoai13aa1n02x7               g234(.a(new_n329), .b(new_n328), .c(new_n291), .d(new_n300), .o1(new_n330));
  and002aa1n02x5               g235(.a(\b[29] ), .b(\a[30] ), .o(new_n331));
  oabi12aa1n02x5               g236(.a(new_n325), .b(\a[30] ), .c(\b[29] ), .out0(new_n332));
  oab012aa1n02x4               g237(.a(new_n332), .b(new_n320), .c(new_n331), .out0(new_n333));
  aoi022aa1n03x5               g238(.a(new_n330), .b(new_n325), .c(new_n327), .d(new_n333), .o1(\s[31] ));
  oai012aa1n02x5               g239(.a(new_n100), .b(new_n104), .c(new_n103), .o1(new_n335));
  oaib12aa1n02x5               g240(.a(new_n335), .b(new_n101), .c(new_n99), .out0(new_n336));
  norb02aa1n02x5               g241(.a(new_n336), .b(new_n106), .out0(\s[3] ));
  xorc02aa1n02x5               g242(.a(\a[4] ), .b(\b[3] ), .out0(new_n338));
  norp03aa1n02x5               g243(.a(new_n106), .b(new_n338), .c(new_n101), .o1(new_n339));
  oaoi13aa1n02x5               g244(.a(new_n339), .b(new_n338), .c(new_n106), .d(new_n107), .o1(\s[4] ));
  oab012aa1n02x4               g245(.a(new_n107), .b(new_n105), .c(new_n102), .out0(new_n341));
  aoi012aa1n02x5               g246(.a(new_n110), .b(\a[4] ), .c(\b[3] ), .o1(new_n342));
  nano22aa1n03x7               g247(.a(new_n341), .b(new_n108), .c(new_n342), .out0(new_n343));
  oai022aa1n02x5               g248(.a(new_n106), .b(new_n107), .c(new_n113), .d(new_n112), .o1(new_n344));
  nanb02aa1n02x5               g249(.a(new_n110), .b(new_n108), .out0(new_n345));
  aoi012aa1n02x5               g250(.a(new_n343), .b(new_n344), .c(new_n345), .o1(\s[5] ));
  norb02aa1n02x5               g251(.a(new_n109), .b(new_n127), .out0(new_n347));
  oai012aa1n02x5               g252(.a(new_n347), .b(new_n343), .c(new_n110), .o1(new_n348));
  norp03aa1n02x5               g253(.a(new_n343), .b(new_n347), .c(new_n110), .o1(new_n349));
  norb02aa1n02x5               g254(.a(new_n348), .b(new_n349), .out0(\s[6] ));
  inv000aa1d42x5               g255(.a(new_n127), .o1(new_n351));
  xorc02aa1n02x5               g256(.a(\a[7] ), .b(\b[6] ), .out0(new_n352));
  xnbna2aa1n03x5               g257(.a(new_n352), .b(new_n348), .c(new_n351), .out0(\s[7] ));
  xorc02aa1n02x5               g258(.a(\a[8] ), .b(\b[7] ), .out0(new_n354));
  aob012aa1n03x5               g259(.a(new_n352), .b(new_n348), .c(new_n351), .out0(new_n355));
  nanp02aa1n02x5               g260(.a(new_n355), .b(new_n120), .o1(new_n356));
  aoi022aa1n02x5               g261(.a(new_n115), .b(new_n116), .c(new_n119), .d(new_n118), .o1(new_n357));
  aoi022aa1n02x5               g262(.a(new_n356), .b(new_n354), .c(new_n355), .d(new_n357), .o1(\s[8] ));
  xnbna2aa1n03x5               g263(.a(new_n132), .b(new_n124), .c(new_n129), .out0(\s[9] ));
endmodule


