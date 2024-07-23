// Benchmark "adder" written by ABC on Thu Jul 18 09:26:28 2024

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
    new_n139, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n179, new_n180, new_n181, new_n182, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n214, new_n215, new_n216,
    new_n217, new_n219, new_n220, new_n221, new_n222, new_n223, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n251, new_n252, new_n253, new_n254, new_n255, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n339, new_n340, new_n342, new_n343, new_n345, new_n346, new_n347,
    new_n349, new_n350, new_n352, new_n354;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n10x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n06x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nor002aa1d32x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nor042aa1n04x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nand42aa1d28x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand02aa1d28x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  aoi012aa1n12x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  nand02aa1n08x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  inv040aa1d32x5               g010(.a(\a[4] ), .o1(new_n106));
  inv040aa1n20x5               g011(.a(\b[3] ), .o1(new_n107));
  nand02aa1d10x5               g012(.a(new_n107), .b(new_n106), .o1(new_n108));
  nand42aa1n02x5               g013(.a(new_n108), .b(new_n105), .o1(new_n109));
  inv040aa1d32x5               g014(.a(\a[3] ), .o1(new_n110));
  inv000aa1d48x5               g015(.a(\b[2] ), .o1(new_n111));
  nanp02aa1n09x5               g016(.a(new_n111), .b(new_n110), .o1(new_n112));
  nanp02aa1n06x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nand22aa1n03x5               g018(.a(new_n112), .b(new_n113), .o1(new_n114));
  nor043aa1n02x5               g019(.a(new_n104), .b(new_n109), .c(new_n114), .o1(new_n115));
  nor042aa1n06x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  oaoi03aa1n03x5               g021(.a(new_n106), .b(new_n107), .c(new_n116), .o1(new_n117));
  inv000aa1n02x5               g022(.a(new_n117), .o1(new_n118));
  nor002aa1d24x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nand02aa1n03x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nanb02aa1n09x5               g025(.a(new_n119), .b(new_n120), .out0(new_n121));
  xnrc02aa1n12x5               g026(.a(\b[4] ), .b(\a[5] ), .out0(new_n122));
  nand02aa1d06x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  nor022aa1n16x5               g028(.a(\b[5] ), .b(\a[6] ), .o1(new_n124));
  nor022aa1n16x5               g029(.a(\b[7] ), .b(\a[8] ), .o1(new_n125));
  nand22aa1n12x5               g030(.a(\b[7] ), .b(\a[8] ), .o1(new_n126));
  nona23aa1n09x5               g031(.a(new_n123), .b(new_n126), .c(new_n125), .d(new_n124), .out0(new_n127));
  nor043aa1d12x5               g032(.a(new_n127), .b(new_n122), .c(new_n121), .o1(new_n128));
  oai012aa1n12x5               g033(.a(new_n128), .b(new_n115), .c(new_n118), .o1(new_n129));
  aoi012aa1n02x7               g034(.a(new_n119), .b(\a[8] ), .c(\b[7] ), .o1(new_n130));
  oai022aa1d18x5               g035(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n131));
  oai112aa1n03x5               g036(.a(new_n131), .b(new_n123), .c(\b[7] ), .d(\a[8] ), .o1(new_n132));
  nano22aa1n03x7               g037(.a(new_n132), .b(new_n120), .c(new_n130), .out0(new_n133));
  aoi012aa1n02x5               g038(.a(new_n125), .b(new_n119), .c(new_n126), .o1(new_n134));
  norb02aa1n06x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  nanp02aa1n03x5               g040(.a(new_n129), .b(new_n135), .o1(new_n136));
  xorc02aa1n12x5               g041(.a(\a[9] ), .b(\b[8] ), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n99), .b(new_n100), .c(new_n136), .d(new_n137), .o1(new_n138));
  aoi112aa1n03x5               g043(.a(new_n99), .b(new_n100), .c(new_n136), .d(new_n137), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(\s[10] ));
  nor002aa1d32x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand42aa1n08x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norb02aa1n02x7               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  norb02aa1n02x5               g048(.a(new_n98), .b(new_n139), .out0(new_n144));
  nanb03aa1n02x5               g049(.a(new_n141), .b(new_n142), .c(new_n98), .out0(new_n145));
  nor002aa1n02x5               g050(.a(new_n139), .b(new_n145), .o1(new_n146));
  oab012aa1n02x4               g051(.a(new_n146), .b(new_n144), .c(new_n143), .out0(\s[11] ));
  inv030aa1n06x5               g052(.a(new_n141), .o1(new_n148));
  nor042aa1d18x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nand02aa1d12x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nanb02aa1n03x5               g055(.a(new_n149), .b(new_n150), .out0(new_n151));
  oaoi13aa1n02x5               g056(.a(new_n151), .b(new_n148), .c(new_n139), .d(new_n145), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n151), .b(new_n141), .out0(new_n153));
  aoib12aa1n02x5               g058(.a(new_n152), .b(new_n153), .c(new_n146), .out0(\s[12] ));
  oai013aa1n02x5               g059(.a(new_n117), .b(new_n104), .c(new_n109), .d(new_n114), .o1(new_n155));
  nanb02aa1n02x5               g060(.a(new_n133), .b(new_n134), .out0(new_n156));
  nona23aa1n03x5               g061(.a(new_n150), .b(new_n142), .c(new_n141), .d(new_n149), .out0(new_n157));
  norb03aa1n03x5               g062(.a(new_n137), .b(new_n157), .c(new_n99), .out0(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n156), .c(new_n155), .d(new_n128), .o1(new_n159));
  nona23aa1n09x5               g064(.a(new_n137), .b(new_n143), .c(new_n99), .d(new_n151), .out0(new_n160));
  nanb03aa1n12x5               g065(.a(new_n149), .b(new_n150), .c(new_n142), .out0(new_n161));
  oai112aa1n06x5               g066(.a(new_n148), .b(new_n98), .c(new_n100), .d(new_n97), .o1(new_n162));
  aoi012aa1n09x5               g067(.a(new_n149), .b(new_n141), .c(new_n150), .o1(new_n163));
  oai012aa1d24x5               g068(.a(new_n163), .b(new_n162), .c(new_n161), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  aoai13aa1n06x5               g070(.a(new_n165), .b(new_n160), .c(new_n129), .d(new_n135), .o1(new_n166));
  nor002aa1d32x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  nand42aa1n10x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  oaib12aa1n02x5               g074(.a(new_n163), .b(new_n167), .c(new_n168), .out0(new_n170));
  oab012aa1n02x4               g075(.a(new_n170), .b(new_n162), .c(new_n161), .out0(new_n171));
  aoi022aa1n02x5               g076(.a(new_n166), .b(new_n169), .c(new_n159), .d(new_n171), .o1(\s[13] ));
  nor002aa1d32x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nand42aa1n20x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n167), .c(new_n166), .d(new_n168), .o1(new_n176));
  aoi112aa1n02x5               g081(.a(new_n167), .b(new_n175), .c(new_n166), .d(new_n169), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n176), .b(new_n177), .out0(\s[14] ));
  oai012aa1n02x5               g083(.a(new_n174), .b(new_n173), .c(new_n167), .o1(new_n179));
  nano23aa1n03x5               g084(.a(new_n167), .b(new_n173), .c(new_n174), .d(new_n168), .out0(new_n180));
  nanp02aa1n02x5               g085(.a(new_n166), .b(new_n180), .o1(new_n181));
  xnrc02aa1n12x5               g086(.a(\b[14] ), .b(\a[15] ), .out0(new_n182));
  xobna2aa1n03x5               g087(.a(new_n182), .b(new_n181), .c(new_n179), .out0(\s[15] ));
  tech160nm_fiao0012aa1n02p5x5 g088(.a(new_n182), .b(new_n181), .c(new_n179), .o(new_n184));
  nor022aa1n16x5               g089(.a(\b[14] ), .b(\a[15] ), .o1(new_n185));
  inv040aa1n03x5               g090(.a(new_n185), .o1(new_n186));
  aoai13aa1n02x5               g091(.a(new_n186), .b(new_n182), .c(new_n181), .d(new_n179), .o1(new_n187));
  inv040aa1d32x5               g092(.a(\a[16] ), .o1(new_n188));
  inv040aa1d32x5               g093(.a(\b[15] ), .o1(new_n189));
  nand42aa1n04x5               g094(.a(new_n189), .b(new_n188), .o1(new_n190));
  nand22aa1n12x5               g095(.a(\b[15] ), .b(\a[16] ), .o1(new_n191));
  nanp02aa1n04x5               g096(.a(new_n190), .b(new_n191), .o1(new_n192));
  inv040aa1d32x5               g097(.a(\a[15] ), .o1(new_n193));
  inv040aa1n16x5               g098(.a(\b[14] ), .o1(new_n194));
  aoi022aa1n02x5               g099(.a(new_n190), .b(new_n191), .c(new_n194), .d(new_n193), .o1(new_n195));
  aboi22aa1n03x5               g100(.a(new_n192), .b(new_n187), .c(new_n184), .d(new_n195), .out0(\s[16] ));
  nona22aa1n02x4               g101(.a(new_n180), .b(new_n182), .c(new_n192), .out0(new_n197));
  nor042aa1n02x5               g102(.a(new_n160), .b(new_n197), .o1(new_n198));
  aoai13aa1n02x7               g103(.a(new_n198), .b(new_n156), .c(new_n155), .d(new_n128), .o1(new_n199));
  nona23aa1n09x5               g104(.a(new_n174), .b(new_n168), .c(new_n167), .d(new_n173), .out0(new_n200));
  nor043aa1n09x5               g105(.a(new_n200), .b(new_n182), .c(new_n192), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(new_n158), .b(new_n201), .o1(new_n202));
  oai112aa1n02x5               g107(.a(new_n190), .b(new_n191), .c(new_n194), .d(new_n193), .o1(new_n203));
  oai112aa1n02x5               g108(.a(new_n186), .b(new_n174), .c(new_n173), .d(new_n167), .o1(new_n204));
  oaoi03aa1n02x5               g109(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n205));
  oai012aa1n06x5               g110(.a(new_n205), .b(new_n204), .c(new_n203), .o1(new_n206));
  aoi012aa1d18x5               g111(.a(new_n206), .b(new_n201), .c(new_n164), .o1(new_n207));
  aoai13aa1n12x5               g112(.a(new_n207), .b(new_n202), .c(new_n129), .d(new_n135), .o1(new_n208));
  xorc02aa1n12x5               g113(.a(\a[17] ), .b(\b[16] ), .out0(new_n209));
  norp02aa1n02x5               g114(.a(new_n204), .b(new_n203), .o1(new_n210));
  nanb02aa1n02x5               g115(.a(new_n209), .b(new_n205), .out0(new_n211));
  aoi112aa1n02x5               g116(.a(new_n210), .b(new_n211), .c(new_n201), .d(new_n164), .o1(new_n212));
  aoi022aa1n02x5               g117(.a(new_n208), .b(new_n209), .c(new_n199), .d(new_n212), .o1(\s[17] ));
  inv000aa1d42x5               g118(.a(\a[17] ), .o1(new_n214));
  nanb02aa1n02x5               g119(.a(\b[16] ), .b(new_n214), .out0(new_n215));
  nand22aa1n03x5               g120(.a(new_n208), .b(new_n209), .o1(new_n216));
  xorc02aa1n12x5               g121(.a(\a[18] ), .b(\b[17] ), .out0(new_n217));
  xnbna2aa1n03x5               g122(.a(new_n217), .b(new_n216), .c(new_n215), .out0(\s[18] ));
  oaoi03aa1n02x5               g123(.a(\a[18] ), .b(\b[17] ), .c(new_n215), .o1(new_n219));
  and002aa1n06x5               g124(.a(new_n217), .b(new_n209), .o(new_n220));
  tech160nm_fixorc02aa1n02p5x5 g125(.a(\a[19] ), .b(\b[18] ), .out0(new_n221));
  aoai13aa1n06x5               g126(.a(new_n221), .b(new_n219), .c(new_n208), .d(new_n220), .o1(new_n222));
  aoi112aa1n02x5               g127(.a(new_n219), .b(new_n221), .c(new_n208), .d(new_n220), .o1(new_n223));
  norb02aa1n03x4               g128(.a(new_n222), .b(new_n223), .out0(\s[19] ));
  xnrc02aa1n02x5               g129(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1d32x5               g130(.a(\a[19] ), .o1(new_n226));
  oaib12aa1n03x5               g131(.a(new_n222), .b(\b[18] ), .c(new_n226), .out0(new_n227));
  xorc02aa1n02x5               g132(.a(\a[20] ), .b(\b[19] ), .out0(new_n228));
  inv040aa1d28x5               g133(.a(\b[18] ), .o1(new_n229));
  inv040aa1d32x5               g134(.a(\a[20] ), .o1(new_n230));
  inv040aa1n10x5               g135(.a(\b[19] ), .o1(new_n231));
  nand02aa1n04x5               g136(.a(new_n231), .b(new_n230), .o1(new_n232));
  nand02aa1d08x5               g137(.a(\b[19] ), .b(\a[20] ), .o1(new_n233));
  aoi022aa1n02x5               g138(.a(new_n232), .b(new_n233), .c(new_n229), .d(new_n226), .o1(new_n234));
  aoi022aa1n02x7               g139(.a(new_n227), .b(new_n228), .c(new_n222), .d(new_n234), .o1(\s[20] ));
  nanp03aa1n02x5               g140(.a(new_n220), .b(new_n221), .c(new_n228), .o1(new_n236));
  inv000aa1n02x5               g141(.a(new_n236), .o1(new_n237));
  oai112aa1n04x5               g142(.a(new_n232), .b(new_n233), .c(new_n229), .d(new_n226), .o1(new_n238));
  nand42aa1n02x5               g143(.a(\b[17] ), .b(\a[18] ), .o1(new_n239));
  oai022aa1d18x5               g144(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n240));
  oai112aa1n06x5               g145(.a(new_n240), .b(new_n239), .c(\b[18] ), .d(\a[19] ), .o1(new_n241));
  norp02aa1n02x5               g146(.a(\b[19] ), .b(\a[20] ), .o1(new_n242));
  aoi013aa1n06x4               g147(.a(new_n242), .b(new_n233), .c(new_n226), .d(new_n229), .o1(new_n243));
  oai012aa1n06x5               g148(.a(new_n243), .b(new_n241), .c(new_n238), .o1(new_n244));
  xorc02aa1n06x5               g149(.a(\a[21] ), .b(\b[20] ), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n244), .c(new_n208), .d(new_n237), .o1(new_n246));
  inv000aa1n02x5               g151(.a(new_n245), .o1(new_n247));
  oai112aa1n02x5               g152(.a(new_n243), .b(new_n247), .c(new_n241), .d(new_n238), .o1(new_n248));
  aoi012aa1n02x5               g153(.a(new_n248), .b(new_n208), .c(new_n237), .o1(new_n249));
  norb02aa1n03x4               g154(.a(new_n246), .b(new_n249), .out0(\s[21] ));
  inv000aa1d42x5               g155(.a(\a[21] ), .o1(new_n251));
  oaib12aa1n03x5               g156(.a(new_n246), .b(\b[20] ), .c(new_n251), .out0(new_n252));
  xorc02aa1n02x5               g157(.a(\a[22] ), .b(\b[21] ), .out0(new_n253));
  nor042aa1n03x5               g158(.a(\b[20] ), .b(\a[21] ), .o1(new_n254));
  norp02aa1n02x5               g159(.a(new_n253), .b(new_n254), .o1(new_n255));
  aoi022aa1n02x7               g160(.a(new_n252), .b(new_n253), .c(new_n246), .d(new_n255), .o1(\s[22] ));
  inv000aa1d42x5               g161(.a(\a[22] ), .o1(new_n257));
  xroi22aa1d06x4               g162(.a(new_n251), .b(\b[20] ), .c(new_n257), .d(\b[21] ), .out0(new_n258));
  aob012aa1n02x5               g163(.a(new_n254), .b(\b[21] ), .c(\a[22] ), .out0(new_n259));
  oaib12aa1n09x5               g164(.a(new_n259), .b(\b[21] ), .c(new_n257), .out0(new_n260));
  aoi012aa1n02x5               g165(.a(new_n260), .b(new_n244), .c(new_n258), .o1(new_n261));
  nand23aa1n03x5               g166(.a(new_n208), .b(new_n237), .c(new_n258), .o1(new_n262));
  xorc02aa1n12x5               g167(.a(\a[23] ), .b(\b[22] ), .out0(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n263), .b(new_n262), .c(new_n261), .out0(\s[23] ));
  aob012aa1n03x5               g169(.a(new_n263), .b(new_n262), .c(new_n261), .out0(new_n265));
  nor042aa1n06x5               g170(.a(\b[22] ), .b(\a[23] ), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n263), .o1(new_n268));
  aoai13aa1n03x5               g173(.a(new_n267), .b(new_n268), .c(new_n262), .d(new_n261), .o1(new_n269));
  tech160nm_fixorc02aa1n03p5x5 g174(.a(\a[24] ), .b(\b[23] ), .out0(new_n270));
  norp02aa1n02x5               g175(.a(new_n270), .b(new_n266), .o1(new_n271));
  aoi022aa1n03x5               g176(.a(new_n269), .b(new_n270), .c(new_n265), .d(new_n271), .o1(\s[24] ));
  nano32aa1n02x4               g177(.a(new_n247), .b(new_n270), .c(new_n253), .d(new_n263), .out0(new_n273));
  inv000aa1n02x5               g178(.a(new_n273), .o1(new_n274));
  nona22aa1n06x5               g179(.a(new_n208), .b(new_n236), .c(new_n274), .out0(new_n275));
  and002aa1n03x5               g180(.a(new_n270), .b(new_n263), .o(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n260), .c(new_n244), .d(new_n258), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[24] ), .b(\b[23] ), .c(new_n267), .carry(new_n278));
  and002aa1n02x5               g183(.a(new_n277), .b(new_n278), .o(new_n279));
  xorc02aa1n12x5               g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  aob012aa1n03x5               g185(.a(new_n280), .b(new_n275), .c(new_n279), .out0(new_n281));
  inv000aa1d42x5               g186(.a(new_n280), .o1(new_n282));
  and003aa1n02x5               g187(.a(new_n277), .b(new_n282), .c(new_n278), .o(new_n283));
  aobi12aa1n02x7               g188(.a(new_n281), .b(new_n283), .c(new_n275), .out0(\s[25] ));
  nor042aa1n06x5               g189(.a(\b[24] ), .b(\a[25] ), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n286), .b(new_n282), .c(new_n275), .d(new_n279), .o1(new_n287));
  xorc02aa1n02x5               g192(.a(\a[26] ), .b(\b[25] ), .out0(new_n288));
  norp02aa1n02x5               g193(.a(new_n288), .b(new_n285), .o1(new_n289));
  aoi022aa1n03x5               g194(.a(new_n287), .b(new_n288), .c(new_n281), .d(new_n289), .o1(\s[26] ));
  and002aa1n02x5               g195(.a(new_n288), .b(new_n280), .o(new_n291));
  aobi12aa1d24x5               g196(.a(new_n291), .b(new_n277), .c(new_n278), .out0(new_n292));
  inv000aa1d42x5               g197(.a(new_n292), .o1(new_n293));
  nano32aa1n03x7               g198(.a(new_n236), .b(new_n291), .c(new_n258), .d(new_n276), .out0(new_n294));
  oao003aa1n06x5               g199(.a(\a[26] ), .b(\b[25] ), .c(new_n286), .carry(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  aoi112aa1n09x5               g201(.a(new_n292), .b(new_n296), .c(new_n208), .d(new_n294), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[27] ), .b(\b[26] ), .out0(new_n298));
  aoi112aa1n02x5               g203(.a(new_n298), .b(new_n296), .c(new_n208), .d(new_n294), .o1(new_n299));
  aboi22aa1n03x5               g204(.a(new_n297), .b(new_n298), .c(new_n299), .d(new_n293), .out0(\s[27] ));
  inv000aa1n06x5               g205(.a(new_n294), .o1(new_n301));
  aoai13aa1n02x7               g206(.a(new_n295), .b(new_n301), .c(new_n199), .d(new_n207), .o1(new_n302));
  oai012aa1n02x5               g207(.a(new_n298), .b(new_n302), .c(new_n292), .o1(new_n303));
  oaoi03aa1n03x5               g208(.a(\a[27] ), .b(\b[26] ), .c(new_n297), .o1(new_n304));
  norp02aa1n02x5               g209(.a(\b[27] ), .b(\a[28] ), .o1(new_n305));
  nanp02aa1n02x5               g210(.a(\b[27] ), .b(\a[28] ), .o1(new_n306));
  norb02aa1n06x4               g211(.a(new_n306), .b(new_n305), .out0(new_n307));
  norp02aa1n02x5               g212(.a(\b[26] ), .b(\a[27] ), .o1(new_n308));
  aoib12aa1n02x5               g213(.a(new_n308), .b(new_n306), .c(new_n305), .out0(new_n309));
  aoi022aa1n03x5               g214(.a(new_n304), .b(new_n307), .c(new_n303), .d(new_n309), .o1(\s[28] ));
  and002aa1n02x5               g215(.a(new_n298), .b(new_n307), .o(new_n311));
  oai012aa1n02x5               g216(.a(new_n311), .b(new_n302), .c(new_n292), .o1(new_n312));
  inv040aa1n03x5               g217(.a(new_n311), .o1(new_n313));
  oai012aa1n02x5               g218(.a(new_n306), .b(new_n305), .c(new_n308), .o1(new_n314));
  tech160nm_fioai012aa1n05x5   g219(.a(new_n314), .b(new_n297), .c(new_n313), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .out0(new_n316));
  norb02aa1n02x5               g221(.a(new_n314), .b(new_n316), .out0(new_n317));
  aoi022aa1n03x5               g222(.a(new_n315), .b(new_n316), .c(new_n312), .d(new_n317), .o1(\s[29] ));
  xorb03aa1n02x5               g223(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g224(.a(new_n298), .b(new_n316), .c(new_n307), .o(new_n320));
  oai012aa1n02x5               g225(.a(new_n320), .b(new_n302), .c(new_n292), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n320), .o1(new_n322));
  oao003aa1n02x5               g227(.a(\a[29] ), .b(\b[28] ), .c(new_n314), .carry(new_n323));
  tech160nm_fioai012aa1n05x5   g228(.a(new_n323), .b(new_n297), .c(new_n322), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .out0(new_n325));
  norb02aa1n02x5               g230(.a(new_n323), .b(new_n325), .out0(new_n326));
  aoi022aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n321), .d(new_n326), .o1(\s[30] ));
  nano22aa1n02x4               g232(.a(new_n313), .b(new_n316), .c(new_n325), .out0(new_n328));
  oai012aa1n02x5               g233(.a(new_n328), .b(new_n302), .c(new_n292), .o1(new_n329));
  xorc02aa1n02x5               g234(.a(\a[31] ), .b(\b[30] ), .out0(new_n330));
  and002aa1n02x5               g235(.a(\b[29] ), .b(\a[30] ), .o(new_n331));
  oabi12aa1n02x5               g236(.a(new_n330), .b(\a[30] ), .c(\b[29] ), .out0(new_n332));
  oab012aa1n02x4               g237(.a(new_n332), .b(new_n323), .c(new_n331), .out0(new_n333));
  inv000aa1n02x5               g238(.a(new_n328), .o1(new_n334));
  oao003aa1n02x5               g239(.a(\a[30] ), .b(\b[29] ), .c(new_n323), .carry(new_n335));
  tech160nm_fioai012aa1n05x5   g240(.a(new_n335), .b(new_n297), .c(new_n334), .o1(new_n336));
  aoi022aa1n03x5               g241(.a(new_n336), .b(new_n330), .c(new_n329), .d(new_n333), .o1(\s[31] ));
  xnbna2aa1n03x5               g242(.a(new_n104), .b(new_n113), .c(new_n112), .out0(\s[3] ));
  nanb03aa1n02x5               g243(.a(new_n104), .b(new_n113), .c(new_n112), .out0(new_n339));
  aoi022aa1n02x5               g244(.a(new_n108), .b(new_n105), .c(new_n110), .d(new_n111), .o1(new_n340));
  aoi022aa1n02x5               g245(.a(new_n155), .b(new_n108), .c(new_n340), .d(new_n339), .o1(\s[4] ));
  inv000aa1d42x5               g246(.a(new_n122), .o1(new_n342));
  nano22aa1n02x4               g247(.a(new_n115), .b(new_n117), .c(new_n122), .out0(new_n343));
  aoi012aa1n02x5               g248(.a(new_n343), .b(new_n155), .c(new_n342), .o1(\s[5] ));
  orn002aa1n02x5               g249(.a(\a[5] ), .b(\b[4] ), .o(new_n345));
  norb02aa1n02x5               g250(.a(new_n123), .b(new_n124), .out0(new_n346));
  oai012aa1n02x5               g251(.a(new_n342), .b(new_n115), .c(new_n118), .o1(new_n347));
  xnbna2aa1n03x5               g252(.a(new_n346), .b(new_n347), .c(new_n345), .out0(\s[6] ));
  aobi12aa1n03x5               g253(.a(new_n346), .b(new_n347), .c(new_n345), .out0(new_n349));
  norp02aa1n02x5               g254(.a(new_n349), .b(new_n124), .o1(new_n350));
  xnrb03aa1n02x5               g255(.a(new_n350), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi13aa1n02x5               g256(.a(new_n119), .b(new_n120), .c(new_n349), .d(new_n124), .o1(new_n352));
  xnrb03aa1n03x5               g257(.a(new_n352), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  norb03aa1n02x5               g258(.a(new_n134), .b(new_n133), .c(new_n137), .out0(new_n354));
  aoi022aa1n02x5               g259(.a(new_n136), .b(new_n137), .c(new_n129), .d(new_n354), .o1(\s[9] ));
endmodule


