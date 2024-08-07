// Benchmark "adder" written by ABC on Thu Jul 18 07:12:58 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n327, new_n329, new_n330, new_n331,
    new_n332, new_n333, new_n336, new_n337, new_n338, new_n340, new_n341,
    new_n343, new_n344, new_n345, new_n346, new_n348;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nand42aa1n04x5               g002(.a(\b[3] ), .b(\a[4] ), .o1(new_n98));
  oai022aa1d24x5               g003(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n99));
  nand42aa1n06x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nano32aa1n03x7               g006(.a(new_n99), .b(new_n98), .c(new_n101), .d(new_n100), .out0(new_n102));
  nand22aa1n03x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nor042aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nona22aa1n09x5               g009(.a(new_n100), .b(new_n104), .c(new_n103), .out0(new_n105));
  aoi022aa1n12x5               g010(.a(new_n102), .b(new_n105), .c(new_n99), .d(new_n98), .o1(new_n106));
  nor042aa1n02x5               g011(.a(\b[7] ), .b(\a[8] ), .o1(new_n107));
  nand42aa1n04x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nor002aa1n06x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nand22aa1n04x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nano23aa1n06x5               g015(.a(new_n107), .b(new_n109), .c(new_n110), .d(new_n108), .out0(new_n111));
  tech160nm_fixorc02aa1n04x5   g016(.a(\a[7] ), .b(\b[6] ), .out0(new_n112));
  xorc02aa1n12x5               g017(.a(\a[5] ), .b(\b[4] ), .out0(new_n113));
  nand23aa1n04x5               g018(.a(new_n111), .b(new_n112), .c(new_n113), .o1(new_n114));
  inv020aa1n02x5               g019(.a(new_n109), .o1(new_n115));
  inv000aa1d42x5               g020(.a(\a[7] ), .o1(new_n116));
  inv000aa1d42x5               g021(.a(\b[6] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n117), .b(new_n116), .o1(new_n118));
  and002aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o(new_n119));
  nor002aa1d32x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  nand42aa1n02x5               g025(.a(new_n120), .b(new_n110), .o1(new_n121));
  aoai13aa1n03x5               g026(.a(new_n118), .b(new_n119), .c(new_n121), .d(new_n115), .o1(new_n122));
  aoi012aa1n06x5               g027(.a(new_n107), .b(new_n122), .c(new_n108), .o1(new_n123));
  oai012aa1d24x5               g028(.a(new_n123), .b(new_n106), .c(new_n114), .o1(new_n124));
  tech160nm_fixorc02aa1n03p5x5 g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  nor042aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand02aa1n03x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n03x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  aoai13aa1n09x5               g033(.a(new_n128), .b(new_n97), .c(new_n124), .d(new_n125), .o1(new_n129));
  aoi112aa1n02x5               g034(.a(new_n128), .b(new_n97), .c(new_n124), .d(new_n125), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n129), .b(new_n130), .out0(\s[10] ));
  aoi012aa1n06x5               g036(.a(new_n126), .b(new_n97), .c(new_n127), .o1(new_n132));
  xnrc02aa1n12x5               g037(.a(\b[10] ), .b(\a[11] ), .out0(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n129), .c(new_n132), .out0(\s[11] ));
  nanp02aa1n02x5               g040(.a(new_n129), .b(new_n132), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(new_n136), .b(new_n134), .o1(new_n137));
  nor042aa1n06x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  inv000aa1n02x5               g043(.a(new_n138), .o1(new_n139));
  aoai13aa1n02x5               g044(.a(new_n139), .b(new_n133), .c(new_n129), .d(new_n132), .o1(new_n140));
  xorc02aa1n02x5               g045(.a(\a[12] ), .b(\b[11] ), .out0(new_n141));
  norp02aa1n02x5               g046(.a(new_n141), .b(new_n138), .o1(new_n142));
  aoi022aa1n02x5               g047(.a(new_n140), .b(new_n141), .c(new_n137), .d(new_n142), .o1(\s[12] ));
  nano32aa1n03x7               g048(.a(new_n133), .b(new_n141), .c(new_n125), .d(new_n128), .out0(new_n144));
  tech160nm_fixnrc02aa1n02p5x5 g049(.a(\b[11] ), .b(\a[12] ), .out0(new_n145));
  oao003aa1n02x5               g050(.a(\a[12] ), .b(\b[11] ), .c(new_n139), .carry(new_n146));
  oai013aa1n06x5               g051(.a(new_n146), .b(new_n133), .c(new_n145), .d(new_n132), .o1(new_n147));
  xnrc02aa1n02x5               g052(.a(\b[12] ), .b(\a[13] ), .out0(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n147), .c(new_n124), .d(new_n144), .o1(new_n150));
  aoi112aa1n02x5               g055(.a(new_n149), .b(new_n147), .c(new_n124), .d(new_n144), .o1(new_n151));
  norb02aa1n02x5               g056(.a(new_n150), .b(new_n151), .out0(\s[13] ));
  inv040aa1d28x5               g057(.a(\a[13] ), .o1(new_n153));
  nanb02aa1d24x5               g058(.a(\b[12] ), .b(new_n153), .out0(new_n154));
  xorc02aa1n02x5               g059(.a(\a[14] ), .b(\b[13] ), .out0(new_n155));
  xnbna2aa1n03x5               g060(.a(new_n155), .b(new_n150), .c(new_n154), .out0(\s[14] ));
  inv040aa1d32x5               g061(.a(\a[14] ), .o1(new_n157));
  xroi22aa1d06x4               g062(.a(new_n153), .b(\b[12] ), .c(new_n157), .d(\b[13] ), .out0(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n147), .c(new_n124), .d(new_n144), .o1(new_n159));
  oaoi03aa1n12x5               g064(.a(\a[14] ), .b(\b[13] ), .c(new_n154), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  tech160nm_fixorc02aa1n04x5   g066(.a(\a[15] ), .b(\b[14] ), .out0(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n162), .b(new_n159), .c(new_n161), .out0(\s[15] ));
  tech160nm_finand02aa1n03p5x5 g068(.a(new_n159), .b(new_n161), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(new_n164), .b(new_n162), .o1(new_n165));
  nor042aa1n06x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  tech160nm_fixnrc02aa1n04x5   g072(.a(\b[14] ), .b(\a[15] ), .out0(new_n168));
  aoai13aa1n02x5               g073(.a(new_n167), .b(new_n168), .c(new_n159), .d(new_n161), .o1(new_n169));
  xorc02aa1n12x5               g074(.a(\a[16] ), .b(\b[15] ), .out0(new_n170));
  norp02aa1n02x5               g075(.a(new_n170), .b(new_n166), .o1(new_n171));
  aoi022aa1n03x5               g076(.a(new_n169), .b(new_n170), .c(new_n165), .d(new_n171), .o1(\s[16] ));
  nor042aa1n03x5               g077(.a(new_n145), .b(new_n133), .o1(new_n173));
  tech160nm_fixnrc02aa1n02p5x5 g078(.a(\b[15] ), .b(\a[16] ), .out0(new_n174));
  nor022aa1n04x5               g079(.a(new_n174), .b(new_n168), .o1(new_n175));
  nand02aa1d04x5               g080(.a(new_n158), .b(new_n175), .o1(new_n176));
  nano32aa1d12x5               g081(.a(new_n176), .b(new_n173), .c(new_n128), .d(new_n125), .out0(new_n177));
  nand02aa1d08x5               g082(.a(new_n124), .b(new_n177), .o1(new_n178));
  oaoi03aa1n03x5               g083(.a(\a[16] ), .b(\b[15] ), .c(new_n167), .o1(new_n179));
  aoi013aa1n09x5               g084(.a(new_n179), .b(new_n160), .c(new_n162), .d(new_n170), .o1(new_n180));
  inv040aa1n06x5               g085(.a(new_n180), .o1(new_n181));
  aoi013aa1n06x5               g086(.a(new_n181), .b(new_n147), .c(new_n158), .d(new_n175), .o1(new_n182));
  nand02aa1d10x5               g087(.a(new_n178), .b(new_n182), .o1(new_n183));
  xorc02aa1n12x5               g088(.a(\a[17] ), .b(\b[16] ), .out0(new_n184));
  aoi113aa1n02x5               g089(.a(new_n181), .b(new_n184), .c(new_n147), .d(new_n158), .e(new_n175), .o1(new_n185));
  aoi022aa1n02x5               g090(.a(new_n183), .b(new_n184), .c(new_n178), .d(new_n185), .o1(\s[17] ));
  inv000aa1d42x5               g091(.a(\a[17] ), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(\b[16] ), .b(new_n187), .out0(new_n188));
  nona22aa1n03x5               g093(.a(new_n141), .b(new_n133), .c(new_n132), .out0(new_n189));
  aoai13aa1n12x5               g094(.a(new_n180), .b(new_n176), .c(new_n189), .d(new_n146), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n184), .b(new_n190), .c(new_n124), .d(new_n177), .o1(new_n191));
  nor042aa1n06x5               g096(.a(\b[17] ), .b(\a[18] ), .o1(new_n192));
  nand02aa1d08x5               g097(.a(\b[17] ), .b(\a[18] ), .o1(new_n193));
  norb02aa1n06x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  xnbna2aa1n03x5               g099(.a(new_n194), .b(new_n191), .c(new_n188), .out0(\s[18] ));
  and002aa1n02x5               g100(.a(new_n184), .b(new_n194), .o(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n190), .c(new_n124), .d(new_n177), .o1(new_n197));
  oaoi03aa1n02x5               g102(.a(\a[18] ), .b(\b[17] ), .c(new_n188), .o1(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  nor042aa1d18x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nand02aa1n06x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norb02aa1n12x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n197), .c(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n06x5               g109(.a(new_n202), .b(new_n198), .c(new_n183), .d(new_n196), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n200), .o1(new_n206));
  inv000aa1d42x5               g111(.a(new_n202), .o1(new_n207));
  aoai13aa1n02x5               g112(.a(new_n206), .b(new_n207), .c(new_n197), .d(new_n199), .o1(new_n208));
  nor042aa1n06x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand02aa1d12x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  norb02aa1n03x4               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  inv000aa1d42x5               g116(.a(\a[19] ), .o1(new_n212));
  inv000aa1d42x5               g117(.a(\b[18] ), .o1(new_n213));
  aboi22aa1n03x5               g118(.a(new_n209), .b(new_n210), .c(new_n212), .d(new_n213), .out0(new_n214));
  aoi022aa1n03x5               g119(.a(new_n208), .b(new_n211), .c(new_n205), .d(new_n214), .o1(\s[20] ));
  nano32aa1n03x7               g120(.a(new_n207), .b(new_n184), .c(new_n211), .d(new_n194), .out0(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n190), .c(new_n124), .d(new_n177), .o1(new_n217));
  nanb03aa1n12x5               g122(.a(new_n209), .b(new_n210), .c(new_n201), .out0(new_n218));
  nor042aa1n04x5               g123(.a(\b[16] ), .b(\a[17] ), .o1(new_n219));
  oai112aa1n06x5               g124(.a(new_n206), .b(new_n193), .c(new_n192), .d(new_n219), .o1(new_n220));
  aoi012aa1n12x5               g125(.a(new_n209), .b(new_n200), .c(new_n210), .o1(new_n221));
  oaih12aa1n12x5               g126(.a(new_n221), .b(new_n220), .c(new_n218), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  nor042aa1d18x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  nanp02aa1n04x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  norb02aa1d21x5               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  xnbna2aa1n03x5               g131(.a(new_n226), .b(new_n217), .c(new_n223), .out0(\s[21] ));
  aoai13aa1n06x5               g132(.a(new_n226), .b(new_n222), .c(new_n183), .d(new_n216), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n224), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n226), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n229), .b(new_n230), .c(new_n217), .d(new_n223), .o1(new_n231));
  nor042aa1n04x5               g136(.a(\b[21] ), .b(\a[22] ), .o1(new_n232));
  nanp02aa1n09x5               g137(.a(\b[21] ), .b(\a[22] ), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(new_n234));
  aoib12aa1n02x5               g139(.a(new_n224), .b(new_n233), .c(new_n232), .out0(new_n235));
  aoi022aa1n02x5               g140(.a(new_n231), .b(new_n234), .c(new_n228), .d(new_n235), .o1(\s[22] ));
  inv000aa1n02x5               g141(.a(new_n216), .o1(new_n237));
  nano22aa1n02x4               g142(.a(new_n237), .b(new_n226), .c(new_n234), .out0(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n190), .c(new_n124), .d(new_n177), .o1(new_n239));
  nano23aa1n09x5               g144(.a(new_n224), .b(new_n232), .c(new_n233), .d(new_n225), .out0(new_n240));
  aoi012aa1d18x5               g145(.a(new_n232), .b(new_n224), .c(new_n233), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  aoi012aa1n02x5               g147(.a(new_n242), .b(new_n222), .c(new_n240), .o1(new_n243));
  nand02aa1d04x5               g148(.a(new_n239), .b(new_n243), .o1(new_n244));
  xorc02aa1n12x5               g149(.a(\a[23] ), .b(\b[22] ), .out0(new_n245));
  aoi112aa1n02x5               g150(.a(new_n245), .b(new_n242), .c(new_n222), .d(new_n240), .o1(new_n246));
  aoi022aa1n02x5               g151(.a(new_n244), .b(new_n245), .c(new_n239), .d(new_n246), .o1(\s[23] ));
  nand02aa1n02x5               g152(.a(new_n244), .b(new_n245), .o1(new_n248));
  nor042aa1n06x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n245), .o1(new_n251));
  aoai13aa1n02x7               g156(.a(new_n250), .b(new_n251), .c(new_n239), .d(new_n243), .o1(new_n252));
  tech160nm_fixorc02aa1n02p5x5 g157(.a(\a[24] ), .b(\b[23] ), .out0(new_n253));
  norp02aa1n02x5               g158(.a(new_n253), .b(new_n249), .o1(new_n254));
  aoi022aa1n03x5               g159(.a(new_n252), .b(new_n253), .c(new_n248), .d(new_n254), .o1(\s[24] ));
  and002aa1n12x5               g160(.a(new_n253), .b(new_n245), .o(new_n256));
  nano22aa1n06x5               g161(.a(new_n237), .b(new_n256), .c(new_n240), .out0(new_n257));
  aoai13aa1n06x5               g162(.a(new_n257), .b(new_n190), .c(new_n124), .d(new_n177), .o1(new_n258));
  nano22aa1n02x4               g163(.a(new_n209), .b(new_n201), .c(new_n210), .out0(new_n259));
  oai012aa1n02x5               g164(.a(new_n193), .b(\b[18] ), .c(\a[19] ), .o1(new_n260));
  oab012aa1n02x4               g165(.a(new_n260), .b(new_n219), .c(new_n192), .out0(new_n261));
  inv020aa1n03x5               g166(.a(new_n221), .o1(new_n262));
  aoai13aa1n06x5               g167(.a(new_n240), .b(new_n262), .c(new_n261), .d(new_n259), .o1(new_n263));
  inv000aa1n06x5               g168(.a(new_n256), .o1(new_n264));
  oao003aa1n02x5               g169(.a(\a[24] ), .b(\b[23] ), .c(new_n250), .carry(new_n265));
  aoai13aa1n12x5               g170(.a(new_n265), .b(new_n264), .c(new_n263), .d(new_n241), .o1(new_n266));
  xorc02aa1n12x5               g171(.a(\a[25] ), .b(\b[24] ), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n266), .c(new_n183), .d(new_n257), .o1(new_n268));
  aoai13aa1n06x5               g173(.a(new_n256), .b(new_n242), .c(new_n222), .d(new_n240), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n267), .o1(new_n270));
  and003aa1n02x5               g175(.a(new_n269), .b(new_n270), .c(new_n265), .o(new_n271));
  aobi12aa1n03x7               g176(.a(new_n268), .b(new_n271), .c(new_n258), .out0(\s[25] ));
  inv000aa1d42x5               g177(.a(new_n266), .o1(new_n273));
  nor042aa1n03x5               g178(.a(\b[24] ), .b(\a[25] ), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  aoai13aa1n02x7               g180(.a(new_n275), .b(new_n270), .c(new_n258), .d(new_n273), .o1(new_n276));
  tech160nm_fixorc02aa1n03p5x5 g181(.a(\a[26] ), .b(\b[25] ), .out0(new_n277));
  norp02aa1n02x5               g182(.a(new_n277), .b(new_n274), .o1(new_n278));
  aoi022aa1n03x5               g183(.a(new_n276), .b(new_n277), .c(new_n268), .d(new_n278), .o1(\s[26] ));
  and002aa1n12x5               g184(.a(new_n277), .b(new_n267), .o(new_n280));
  nano32aa1n03x7               g185(.a(new_n237), .b(new_n280), .c(new_n240), .d(new_n256), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n190), .c(new_n124), .d(new_n177), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n280), .o1(new_n283));
  oao003aa1n02x5               g188(.a(\a[26] ), .b(\b[25] ), .c(new_n275), .carry(new_n284));
  aoai13aa1n04x5               g189(.a(new_n284), .b(new_n283), .c(new_n269), .d(new_n265), .o1(new_n285));
  xorc02aa1n12x5               g190(.a(\a[27] ), .b(\b[26] ), .out0(new_n286));
  aoai13aa1n06x5               g191(.a(new_n286), .b(new_n285), .c(new_n183), .d(new_n281), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n284), .o1(new_n288));
  aoi112aa1n02x5               g193(.a(new_n286), .b(new_n288), .c(new_n266), .d(new_n280), .o1(new_n289));
  aobi12aa1n02x7               g194(.a(new_n287), .b(new_n289), .c(new_n282), .out0(\s[27] ));
  tech160nm_fiaoi012aa1n05x5   g195(.a(new_n288), .b(new_n266), .c(new_n280), .o1(new_n291));
  norp02aa1n02x5               g196(.a(\b[26] ), .b(\a[27] ), .o1(new_n292));
  inv000aa1n03x5               g197(.a(new_n292), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n286), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n293), .b(new_n294), .c(new_n291), .d(new_n282), .o1(new_n295));
  tech160nm_fixorc02aa1n03p5x5 g200(.a(\a[28] ), .b(\b[27] ), .out0(new_n296));
  norp02aa1n02x5               g201(.a(new_n296), .b(new_n292), .o1(new_n297));
  aoi022aa1n03x5               g202(.a(new_n295), .b(new_n296), .c(new_n287), .d(new_n297), .o1(\s[28] ));
  and002aa1n02x5               g203(.a(new_n296), .b(new_n286), .o(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n285), .c(new_n183), .d(new_n281), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n299), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[28] ), .b(\b[27] ), .c(new_n293), .carry(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n301), .c(new_n291), .d(new_n282), .o1(new_n303));
  tech160nm_fixorc02aa1n03p5x5 g208(.a(\a[29] ), .b(\b[28] ), .out0(new_n304));
  norb02aa1n02x5               g209(.a(new_n302), .b(new_n304), .out0(new_n305));
  aoi022aa1n03x5               g210(.a(new_n303), .b(new_n304), .c(new_n300), .d(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g211(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g212(.a(new_n294), .b(new_n296), .c(new_n304), .out0(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n285), .c(new_n183), .d(new_n281), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n308), .o1(new_n310));
  oaoi03aa1n02x5               g215(.a(\a[29] ), .b(\b[28] ), .c(new_n302), .o1(new_n311));
  inv000aa1n03x5               g216(.a(new_n311), .o1(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n310), .c(new_n291), .d(new_n282), .o1(new_n313));
  xorc02aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .out0(new_n314));
  and002aa1n02x5               g219(.a(\b[28] ), .b(\a[29] ), .o(new_n315));
  oabi12aa1n02x5               g220(.a(new_n314), .b(\a[29] ), .c(\b[28] ), .out0(new_n316));
  oab012aa1n02x4               g221(.a(new_n316), .b(new_n302), .c(new_n315), .out0(new_n317));
  aoi022aa1n03x5               g222(.a(new_n313), .b(new_n314), .c(new_n309), .d(new_n317), .o1(\s[30] ));
  nano32aa1n02x5               g223(.a(new_n294), .b(new_n314), .c(new_n296), .d(new_n304), .out0(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n285), .c(new_n183), .d(new_n281), .o1(new_n320));
  inv000aa1d42x5               g225(.a(new_n319), .o1(new_n321));
  oao003aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .c(new_n312), .carry(new_n322));
  aoai13aa1n03x5               g227(.a(new_n322), .b(new_n321), .c(new_n291), .d(new_n282), .o1(new_n323));
  xorc02aa1n02x5               g228(.a(\a[31] ), .b(\b[30] ), .out0(new_n324));
  norb02aa1n02x5               g229(.a(new_n322), .b(new_n324), .out0(new_n325));
  aoi022aa1n03x5               g230(.a(new_n323), .b(new_n324), .c(new_n320), .d(new_n325), .o1(\s[31] ));
  oai012aa1n02x5               g231(.a(new_n100), .b(new_n104), .c(new_n103), .o1(new_n327));
  xnrb03aa1n02x5               g232(.a(new_n327), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanb02aa1n02x5               g233(.a(new_n99), .b(new_n98), .out0(new_n329));
  xorc02aa1n02x5               g234(.a(\a[3] ), .b(\b[2] ), .out0(new_n330));
  norp02aa1n02x5               g235(.a(\b[3] ), .b(\a[4] ), .o1(new_n331));
  aboi22aa1n03x5               g236(.a(new_n331), .b(new_n98), .c(\b[2] ), .d(\a[3] ), .out0(new_n332));
  aob012aa1n02x5               g237(.a(new_n332), .b(new_n330), .c(new_n327), .out0(new_n333));
  aoai13aa1n02x5               g238(.a(new_n333), .b(new_n329), .c(new_n105), .d(new_n102), .o1(\s[4] ));
  xnrc02aa1n02x5               g239(.a(new_n106), .b(new_n113), .out0(\s[5] ));
  norb02aa1n02x5               g240(.a(new_n110), .b(new_n109), .out0(new_n336));
  inv000aa1d42x5               g241(.a(new_n120), .o1(new_n337));
  nanb02aa1n02x5               g242(.a(new_n106), .b(new_n113), .out0(new_n338));
  xnbna2aa1n03x5               g243(.a(new_n336), .b(new_n338), .c(new_n337), .out0(\s[6] ));
  aoi012aa1n02x5               g244(.a(new_n109), .b(new_n120), .c(new_n110), .o1(new_n340));
  aob012aa1n02x5               g245(.a(new_n336), .b(new_n338), .c(new_n337), .out0(new_n341));
  xnbna2aa1n03x5               g246(.a(new_n112), .b(new_n341), .c(new_n340), .out0(\s[7] ));
  norb02aa1n02x5               g247(.a(new_n108), .b(new_n107), .out0(new_n343));
  aob012aa1n02x5               g248(.a(new_n112), .b(new_n341), .c(new_n340), .out0(new_n344));
  aoai13aa1n02x5               g249(.a(new_n118), .b(new_n119), .c(new_n341), .d(new_n340), .o1(new_n345));
  aboi22aa1n03x5               g250(.a(new_n107), .b(new_n108), .c(new_n116), .d(new_n117), .out0(new_n346));
  aoi022aa1n02x5               g251(.a(new_n345), .b(new_n343), .c(new_n344), .d(new_n346), .o1(\s[8] ));
  orn002aa1n02x5               g252(.a(new_n106), .b(new_n114), .o(new_n348));
  xnbna2aa1n03x5               g253(.a(new_n125), .b(new_n348), .c(new_n123), .out0(\s[9] ));
endmodule


