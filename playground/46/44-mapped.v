// Benchmark "adder" written by ABC on Thu Jul 18 11:58:45 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n331, new_n332, new_n334, new_n335, new_n337, new_n339, new_n340,
    new_n341, new_n342, new_n344, new_n345, new_n347, new_n348, new_n349,
    new_n350, new_n352, new_n353;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  orn002aa1n02x7               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nand22aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  aoi012aa1d24x5               g005(.a(new_n100), .b(\a[2] ), .c(\b[1] ), .o1(new_n101));
  nand02aa1n08x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nor022aa1n16x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand02aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanb03aa1n06x5               g009(.a(new_n103), .b(new_n104), .c(new_n102), .out0(new_n105));
  oab012aa1n02x5               g010(.a(new_n103), .b(\a[4] ), .c(\b[3] ), .out0(new_n106));
  aoai13aa1n06x5               g011(.a(new_n106), .b(new_n105), .c(new_n99), .d(new_n101), .o1(new_n107));
  nand42aa1n20x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nand42aa1d28x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  oai112aa1n03x5               g014(.a(new_n108), .b(new_n109), .c(\b[7] ), .d(\a[8] ), .o1(new_n110));
  oai022aa1n02x5               g015(.a(\a[6] ), .b(\b[5] ), .c(\b[6] ), .d(\a[7] ), .o1(new_n111));
  aoi022aa1d24x5               g016(.a(\b[6] ), .b(\a[7] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n112));
  inv040aa1d32x5               g017(.a(\a[5] ), .o1(new_n113));
  inv040aa1d32x5               g018(.a(\b[4] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(new_n114), .b(new_n113), .o1(new_n115));
  nand02aa1n03x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand23aa1n02x5               g021(.a(new_n112), .b(new_n115), .c(new_n116), .o1(new_n117));
  nor043aa1n04x5               g022(.a(new_n117), .b(new_n111), .c(new_n110), .o1(new_n118));
  nor002aa1n02x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  inv000aa1n04x5               g024(.a(new_n119), .o1(new_n120));
  norp02aa1n04x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  inv030aa1n04x5               g026(.a(new_n121), .o1(new_n122));
  nor002aa1d32x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  aoai13aa1n12x5               g028(.a(new_n108), .b(new_n123), .c(new_n114), .d(new_n113), .o1(new_n124));
  aoi022aa1n12x5               g029(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n125));
  inv040aa1n04x5               g030(.a(new_n125), .o1(new_n126));
  aoai13aa1n06x5               g031(.a(new_n120), .b(new_n126), .c(new_n124), .d(new_n122), .o1(new_n127));
  xnrc02aa1n12x5               g032(.a(\b[8] ), .b(\a[9] ), .out0(new_n128));
  inv000aa1d42x5               g033(.a(new_n128), .o1(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n127), .c(new_n107), .d(new_n118), .o1(new_n130));
  nor002aa1d32x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand22aa1n12x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nanb02aa1n12x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n130), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n06x5               g040(.a(new_n107), .b(new_n118), .o1(new_n136));
  inv040aa1n02x5               g041(.a(new_n127), .o1(new_n137));
  nanp02aa1n06x5               g042(.a(new_n136), .b(new_n137), .o1(new_n138));
  aoai13aa1n06x5               g043(.a(new_n134), .b(new_n97), .c(new_n138), .d(new_n129), .o1(new_n139));
  aoi012aa1n09x5               g044(.a(new_n131), .b(new_n97), .c(new_n132), .o1(new_n140));
  nor002aa1d32x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand02aa1d06x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  nanb02aa1n02x5               g047(.a(new_n141), .b(new_n142), .out0(new_n143));
  xobna2aa1n03x5               g048(.a(new_n143), .b(new_n139), .c(new_n140), .out0(\s[11] ));
  nor002aa1d24x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nand02aa1d06x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nanb02aa1n02x5               g051(.a(new_n145), .b(new_n146), .out0(new_n147));
  inv040aa1n06x5               g052(.a(new_n141), .o1(new_n148));
  aoai13aa1n02x7               g053(.a(new_n148), .b(new_n143), .c(new_n139), .d(new_n140), .o1(new_n149));
  norb03aa1n02x5               g054(.a(new_n146), .b(new_n141), .c(new_n145), .out0(new_n150));
  aoai13aa1n02x7               g055(.a(new_n150), .b(new_n143), .c(new_n139), .d(new_n140), .o1(new_n151));
  aob012aa1n03x5               g056(.a(new_n151), .b(new_n149), .c(new_n147), .out0(\s[12] ));
  nona23aa1n09x5               g057(.a(new_n146), .b(new_n142), .c(new_n141), .d(new_n145), .out0(new_n153));
  nor003aa1n02x5               g058(.a(new_n153), .b(new_n133), .c(new_n128), .o1(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n127), .c(new_n107), .d(new_n118), .o1(new_n155));
  oaoi03aa1n09x5               g060(.a(\a[12] ), .b(\b[11] ), .c(new_n148), .o1(new_n156));
  oabi12aa1n02x7               g061(.a(new_n156), .b(new_n153), .c(new_n140), .out0(new_n157));
  nanb02aa1n06x5               g062(.a(new_n157), .b(new_n155), .out0(new_n158));
  nor002aa1d32x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nand22aa1n12x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  norb02aa1n06x4               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  inv020aa1n03x5               g066(.a(new_n140), .o1(new_n162));
  nano23aa1n03x7               g067(.a(new_n141), .b(new_n145), .c(new_n146), .d(new_n142), .out0(new_n163));
  aoi112aa1n02x5               g068(.a(new_n156), .b(new_n161), .c(new_n163), .d(new_n162), .o1(new_n164));
  aoi022aa1n02x5               g069(.a(new_n158), .b(new_n161), .c(new_n155), .d(new_n164), .o1(\s[13] ));
  nor042aa1n12x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nand22aa1n06x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  norb02aa1n03x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  aoi012aa1n02x5               g073(.a(new_n159), .b(new_n158), .c(new_n160), .o1(new_n169));
  aoai13aa1n03x5               g074(.a(new_n161), .b(new_n157), .c(new_n138), .d(new_n154), .o1(new_n170));
  nona23aa1n02x4               g075(.a(new_n170), .b(new_n167), .c(new_n166), .d(new_n159), .out0(new_n171));
  oai012aa1n02x5               g076(.a(new_n171), .b(new_n169), .c(new_n168), .o1(\s[14] ));
  nano23aa1n06x5               g077(.a(new_n159), .b(new_n166), .c(new_n167), .d(new_n160), .out0(new_n173));
  oai012aa1n02x7               g078(.a(new_n167), .b(new_n166), .c(new_n159), .o1(new_n174));
  inv000aa1n02x5               g079(.a(new_n174), .o1(new_n175));
  nor002aa1d32x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nand22aa1n04x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  aoai13aa1n06x5               g083(.a(new_n178), .b(new_n175), .c(new_n158), .d(new_n173), .o1(new_n179));
  aoi112aa1n02x5               g084(.a(new_n178), .b(new_n175), .c(new_n158), .d(new_n173), .o1(new_n180));
  norb02aa1n03x4               g085(.a(new_n179), .b(new_n180), .out0(\s[15] ));
  inv000aa1d42x5               g086(.a(new_n176), .o1(new_n182));
  nor002aa1d32x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nand02aa1d08x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  norb02aa1n02x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  nona23aa1n03x5               g090(.a(new_n179), .b(new_n184), .c(new_n183), .d(new_n176), .out0(new_n186));
  aoai13aa1n03x5               g091(.a(new_n186), .b(new_n185), .c(new_n179), .d(new_n182), .o1(\s[16] ));
  nona23aa1n09x5               g092(.a(new_n184), .b(new_n177), .c(new_n176), .d(new_n183), .out0(new_n188));
  inv000aa1n02x5               g093(.a(new_n188), .o1(new_n189));
  aoai13aa1n04x5               g094(.a(new_n189), .b(new_n175), .c(new_n157), .d(new_n173), .o1(new_n190));
  tech160nm_fiaoi012aa1n05x5   g095(.a(new_n127), .b(new_n107), .c(new_n118), .o1(new_n191));
  nano22aa1n02x4               g096(.a(new_n188), .b(new_n161), .c(new_n168), .out0(new_n192));
  nona32aa1n03x5               g097(.a(new_n192), .b(new_n153), .c(new_n133), .d(new_n128), .out0(new_n193));
  tech160nm_fiaoi012aa1n03p5x5 g098(.a(new_n183), .b(new_n176), .c(new_n184), .o1(new_n194));
  oai112aa1n06x5               g099(.a(new_n190), .b(new_n194), .c(new_n193), .d(new_n191), .o1(new_n195));
  xorc02aa1n12x5               g100(.a(\a[17] ), .b(\b[16] ), .out0(new_n196));
  nanb02aa1n02x5               g101(.a(new_n196), .b(new_n194), .out0(new_n197));
  oab012aa1n02x4               g102(.a(new_n197), .b(new_n191), .c(new_n193), .out0(new_n198));
  aoi022aa1n02x5               g103(.a(new_n195), .b(new_n196), .c(new_n198), .d(new_n190), .o1(\s[17] ));
  nor002aa1d32x5               g104(.a(\b[16] ), .b(\a[17] ), .o1(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  aoai13aa1n02x7               g106(.a(new_n173), .b(new_n156), .c(new_n163), .d(new_n162), .o1(new_n202));
  tech160nm_fiaoi012aa1n03p5x5 g107(.a(new_n188), .b(new_n202), .c(new_n174), .o1(new_n203));
  aoai13aa1n04x5               g108(.a(new_n194), .b(new_n193), .c(new_n136), .d(new_n137), .o1(new_n204));
  oaih12aa1n02x5               g109(.a(new_n196), .b(new_n204), .c(new_n203), .o1(new_n205));
  nor042aa1n04x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nand02aa1d28x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  norb02aa1n03x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  nona23aa1n03x5               g113(.a(new_n205), .b(new_n207), .c(new_n206), .d(new_n200), .out0(new_n209));
  aoai13aa1n03x5               g114(.a(new_n209), .b(new_n208), .c(new_n201), .d(new_n205), .o1(\s[18] ));
  and002aa1n02x5               g115(.a(new_n196), .b(new_n208), .o(new_n211));
  aoi012aa1n09x5               g116(.a(new_n206), .b(new_n200), .c(new_n207), .o1(new_n212));
  inv040aa1n02x5               g117(.a(new_n212), .o1(new_n213));
  nor002aa1d32x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nand22aa1n06x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n213), .c(new_n195), .d(new_n211), .o1(new_n217));
  aoi112aa1n02x5               g122(.a(new_n216), .b(new_n213), .c(new_n195), .d(new_n211), .o1(new_n218));
  norb02aa1n03x4               g123(.a(new_n217), .b(new_n218), .out0(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1n06x5               g125(.a(new_n214), .o1(new_n221));
  nor002aa1d32x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  nand02aa1n06x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  norb03aa1n02x5               g129(.a(new_n223), .b(new_n214), .c(new_n222), .out0(new_n225));
  nanp02aa1n03x5               g130(.a(new_n217), .b(new_n225), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n226), .b(new_n224), .c(new_n217), .d(new_n221), .o1(\s[20] ));
  nona23aa1d18x5               g132(.a(new_n223), .b(new_n215), .c(new_n214), .d(new_n222), .out0(new_n228));
  nano22aa1n03x7               g133(.a(new_n228), .b(new_n196), .c(new_n208), .out0(new_n229));
  oaoi03aa1n09x5               g134(.a(\a[20] ), .b(\b[19] ), .c(new_n221), .o1(new_n230));
  oabi12aa1n06x5               g135(.a(new_n230), .b(new_n228), .c(new_n212), .out0(new_n231));
  xnrc02aa1n12x5               g136(.a(\b[20] ), .b(\a[21] ), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n231), .c(new_n195), .d(new_n229), .o1(new_n234));
  nano23aa1n06x5               g139(.a(new_n214), .b(new_n222), .c(new_n223), .d(new_n215), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(new_n230), .b(new_n233), .c(new_n235), .d(new_n213), .o1(new_n236));
  aobi12aa1n02x5               g141(.a(new_n236), .b(new_n195), .c(new_n229), .out0(new_n237));
  norb02aa1n03x4               g142(.a(new_n234), .b(new_n237), .out0(\s[21] ));
  nor042aa1n06x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  inv000aa1n06x5               g144(.a(new_n239), .o1(new_n240));
  xnrc02aa1n12x5               g145(.a(\b[21] ), .b(\a[22] ), .out0(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  oai022aa1n02x5               g147(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n243));
  aoi012aa1n02x5               g148(.a(new_n243), .b(\a[22] ), .c(\b[21] ), .o1(new_n244));
  nanp02aa1n03x5               g149(.a(new_n234), .b(new_n244), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n245), .b(new_n242), .c(new_n234), .d(new_n240), .o1(\s[22] ));
  nor042aa1n06x5               g151(.a(new_n241), .b(new_n232), .o1(new_n247));
  nano32aa1n02x4               g152(.a(new_n228), .b(new_n247), .c(new_n196), .d(new_n208), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n247), .b(new_n230), .c(new_n235), .d(new_n213), .o1(new_n249));
  oao003aa1n02x5               g154(.a(\a[22] ), .b(\b[21] ), .c(new_n240), .carry(new_n250));
  nanp02aa1n02x5               g155(.a(new_n249), .b(new_n250), .o1(new_n251));
  xorc02aa1n12x5               g156(.a(\a[23] ), .b(\b[22] ), .out0(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n251), .c(new_n195), .d(new_n248), .o1(new_n253));
  inv000aa1n02x5               g158(.a(new_n250), .o1(new_n254));
  aoi112aa1n02x5               g159(.a(new_n252), .b(new_n254), .c(new_n231), .d(new_n247), .o1(new_n255));
  aobi12aa1n02x5               g160(.a(new_n255), .b(new_n195), .c(new_n248), .out0(new_n256));
  norb02aa1n03x4               g161(.a(new_n253), .b(new_n256), .out0(\s[23] ));
  nor042aa1n09x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  nor002aa1n04x5               g164(.a(\b[23] ), .b(\a[24] ), .o1(new_n260));
  and002aa1n12x5               g165(.a(\b[23] ), .b(\a[24] ), .o(new_n261));
  nor042aa1n04x5               g166(.a(new_n261), .b(new_n260), .o1(new_n262));
  norp03aa1n02x5               g167(.a(new_n261), .b(new_n260), .c(new_n258), .o1(new_n263));
  nanp02aa1n03x5               g168(.a(new_n253), .b(new_n263), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n262), .c(new_n253), .d(new_n259), .o1(\s[24] ));
  inv000aa1n02x5               g170(.a(new_n229), .o1(new_n266));
  and002aa1n02x5               g171(.a(new_n252), .b(new_n262), .o(new_n267));
  nano22aa1n02x4               g172(.a(new_n266), .b(new_n247), .c(new_n267), .out0(new_n268));
  oai012aa1n02x5               g173(.a(new_n268), .b(new_n204), .c(new_n203), .o1(new_n269));
  inv000aa1n02x5               g174(.a(new_n267), .o1(new_n270));
  oab012aa1n04x5               g175(.a(new_n260), .b(new_n259), .c(new_n261), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n270), .c(new_n249), .d(new_n250), .o1(new_n272));
  xorc02aa1n12x5               g177(.a(\a[25] ), .b(\b[24] ), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n272), .c(new_n195), .d(new_n268), .o1(new_n274));
  aoai13aa1n02x5               g179(.a(new_n267), .b(new_n254), .c(new_n231), .d(new_n247), .o1(new_n275));
  nano22aa1n02x4               g180(.a(new_n273), .b(new_n275), .c(new_n271), .out0(new_n276));
  aobi12aa1n02x7               g181(.a(new_n274), .b(new_n276), .c(new_n269), .out0(\s[25] ));
  nor042aa1n03x5               g182(.a(\b[24] ), .b(\a[25] ), .o1(new_n278));
  inv000aa1n02x5               g183(.a(new_n278), .o1(new_n279));
  norp02aa1n04x5               g184(.a(\b[25] ), .b(\a[26] ), .o1(new_n280));
  and002aa1n12x5               g185(.a(\b[25] ), .b(\a[26] ), .o(new_n281));
  nor042aa1n02x5               g186(.a(new_n281), .b(new_n280), .o1(new_n282));
  norp03aa1n02x5               g187(.a(new_n281), .b(new_n280), .c(new_n278), .o1(new_n283));
  nanp02aa1n03x5               g188(.a(new_n274), .b(new_n283), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n282), .c(new_n274), .d(new_n279), .o1(\s[26] ));
  and002aa1n06x5               g190(.a(new_n273), .b(new_n282), .o(new_n286));
  nano32aa1n03x7               g191(.a(new_n266), .b(new_n286), .c(new_n247), .d(new_n267), .out0(new_n287));
  tech160nm_fioai012aa1n04x5   g192(.a(new_n287), .b(new_n204), .c(new_n203), .o1(new_n288));
  inv000aa1n02x5               g193(.a(new_n286), .o1(new_n289));
  oab012aa1n04x5               g194(.a(new_n280), .b(new_n279), .c(new_n281), .out0(new_n290));
  aoai13aa1n02x7               g195(.a(new_n290), .b(new_n289), .c(new_n275), .d(new_n271), .o1(new_n291));
  xorc02aa1n12x5               g196(.a(\a[27] ), .b(\b[26] ), .out0(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n291), .c(new_n195), .d(new_n287), .o1(new_n293));
  inv000aa1n02x5               g198(.a(new_n290), .o1(new_n294));
  aoi112aa1n02x5               g199(.a(new_n292), .b(new_n294), .c(new_n272), .d(new_n286), .o1(new_n295));
  aobi12aa1n02x7               g200(.a(new_n293), .b(new_n295), .c(new_n288), .out0(\s[27] ));
  norp02aa1n02x5               g201(.a(\b[26] ), .b(\a[27] ), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  xorc02aa1n12x5               g203(.a(\a[28] ), .b(\b[27] ), .out0(new_n299));
  tech160nm_fiaoi012aa1n03p5x5 g204(.a(new_n294), .b(new_n272), .c(new_n286), .o1(new_n300));
  inv040aa1n02x5               g205(.a(new_n292), .o1(new_n301));
  oai022aa1d18x5               g206(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(\a[28] ), .c(\b[27] ), .o1(new_n303));
  aoai13aa1n02x5               g208(.a(new_n303), .b(new_n301), .c(new_n288), .d(new_n300), .o1(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n299), .c(new_n293), .d(new_n298), .o1(\s[28] ));
  and002aa1n02x5               g210(.a(new_n299), .b(new_n292), .o(new_n306));
  aoai13aa1n02x7               g211(.a(new_n306), .b(new_n291), .c(new_n195), .d(new_n287), .o1(new_n307));
  inv000aa1n02x5               g212(.a(new_n306), .o1(new_n308));
  aob012aa1n02x5               g213(.a(new_n302), .b(\b[27] ), .c(\a[28] ), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n309), .b(new_n308), .c(new_n288), .d(new_n300), .o1(new_n310));
  xorc02aa1n12x5               g215(.a(\a[29] ), .b(\b[28] ), .out0(new_n311));
  norb02aa1n02x5               g216(.a(new_n309), .b(new_n311), .out0(new_n312));
  aoi022aa1n03x5               g217(.a(new_n310), .b(new_n311), .c(new_n307), .d(new_n312), .o1(\s[29] ));
  xorb03aa1n02x5               g218(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g219(.a(new_n301), .b(new_n299), .c(new_n311), .out0(new_n315));
  aoai13aa1n02x7               g220(.a(new_n315), .b(new_n291), .c(new_n195), .d(new_n287), .o1(new_n316));
  inv000aa1n02x5               g221(.a(new_n315), .o1(new_n317));
  oao003aa1n03x5               g222(.a(\a[29] ), .b(\b[28] ), .c(new_n309), .carry(new_n318));
  aoai13aa1n02x5               g223(.a(new_n318), .b(new_n317), .c(new_n288), .d(new_n300), .o1(new_n319));
  xorc02aa1n02x5               g224(.a(\a[30] ), .b(\b[29] ), .out0(new_n320));
  norb02aa1n02x5               g225(.a(new_n318), .b(new_n320), .out0(new_n321));
  aoi022aa1n03x5               g226(.a(new_n319), .b(new_n320), .c(new_n316), .d(new_n321), .o1(\s[30] ));
  nano32aa1n06x5               g227(.a(new_n301), .b(new_n320), .c(new_n299), .d(new_n311), .out0(new_n323));
  aoai13aa1n02x7               g228(.a(new_n323), .b(new_n291), .c(new_n195), .d(new_n287), .o1(new_n324));
  inv000aa1n02x5               g229(.a(new_n323), .o1(new_n325));
  oao003aa1n02x5               g230(.a(\a[30] ), .b(\b[29] ), .c(new_n318), .carry(new_n326));
  aoai13aa1n02x5               g231(.a(new_n326), .b(new_n325), .c(new_n288), .d(new_n300), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[31] ), .b(\b[30] ), .out0(new_n328));
  norb02aa1n02x5               g233(.a(new_n326), .b(new_n328), .out0(new_n329));
  aoi022aa1n03x5               g234(.a(new_n327), .b(new_n328), .c(new_n324), .d(new_n329), .o1(\s[31] ));
  nanp02aa1n02x5               g235(.a(new_n101), .b(new_n99), .o1(new_n331));
  norb02aa1n02x5               g236(.a(new_n104), .b(new_n103), .out0(new_n332));
  xobna2aa1n03x5               g237(.a(new_n332), .b(new_n331), .c(new_n102), .out0(\s[3] ));
  xorc02aa1n02x5               g238(.a(\a[4] ), .b(\b[3] ), .out0(new_n334));
  aoi113aa1n02x5               g239(.a(new_n334), .b(new_n103), .c(new_n331), .d(new_n104), .e(new_n102), .o1(new_n335));
  aoi012aa1n02x5               g240(.a(new_n335), .b(new_n107), .c(new_n334), .o1(\s[4] ));
  xnrc02aa1n02x5               g241(.a(\b[4] ), .b(\a[5] ), .out0(new_n337));
  xnbna2aa1n03x5               g242(.a(new_n337), .b(new_n107), .c(new_n109), .out0(\s[5] ));
  nano22aa1n03x7               g243(.a(new_n337), .b(new_n107), .c(new_n109), .out0(new_n339));
  norb02aa1n02x5               g244(.a(new_n108), .b(new_n123), .out0(new_n340));
  aoai13aa1n06x5               g245(.a(new_n340), .b(new_n339), .c(new_n113), .d(new_n114), .o1(new_n341));
  aoi112aa1n02x5               g246(.a(new_n339), .b(new_n340), .c(new_n113), .d(new_n114), .o1(new_n342));
  norb02aa1n02x5               g247(.a(new_n341), .b(new_n342), .out0(\s[6] ));
  and002aa1n02x5               g248(.a(\b[6] ), .b(\a[7] ), .o(new_n344));
  norp02aa1n02x5               g249(.a(new_n344), .b(new_n121), .o1(new_n345));
  xnbna2aa1n03x5               g250(.a(new_n345), .b(new_n341), .c(new_n124), .out0(\s[7] ));
  aob012aa1n03x5               g251(.a(new_n345), .b(new_n341), .c(new_n124), .out0(new_n347));
  aoai13aa1n02x7               g252(.a(new_n122), .b(new_n344), .c(new_n341), .d(new_n124), .o1(new_n348));
  norb02aa1n02x5               g253(.a(new_n116), .b(new_n119), .out0(new_n349));
  aoi012aa1n02x5               g254(.a(new_n121), .b(new_n120), .c(new_n116), .o1(new_n350));
  aoi022aa1n03x5               g255(.a(new_n348), .b(new_n349), .c(new_n347), .d(new_n350), .o1(\s[8] ));
  nanp02aa1n02x5               g256(.a(new_n124), .b(new_n122), .o1(new_n352));
  aoi112aa1n02x5               g257(.a(new_n129), .b(new_n119), .c(new_n352), .d(new_n125), .o1(new_n353));
  aoi022aa1n02x5               g258(.a(new_n138), .b(new_n129), .c(new_n136), .d(new_n353), .o1(\s[9] ));
endmodule


