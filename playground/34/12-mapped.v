// Benchmark "adder" written by ABC on Thu Jul 18 05:27:44 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n336, new_n338, new_n340,
    new_n341, new_n343, new_n344, new_n345, new_n346, new_n348, new_n350,
    new_n351, new_n352;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nand42aa1n03x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  orn002aa1n03x5               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nanp02aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n12x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  aob012aa1n06x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  nor042aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n12x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n06x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nor042aa1n06x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand22aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  norb02aa1n03x5               g012(.a(new_n107), .b(new_n106), .out0(new_n108));
  nand23aa1n06x5               g013(.a(new_n102), .b(new_n105), .c(new_n108), .o1(new_n109));
  aoi012aa1n09x5               g014(.a(new_n103), .b(new_n106), .c(new_n104), .o1(new_n110));
  nor042aa1n04x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nand22aa1n09x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor042aa1d18x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nano23aa1n03x7               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  nor042aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand22aa1n09x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nor042aa1n04x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nand22aa1n12x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nano23aa1n06x5               g024(.a(new_n116), .b(new_n118), .c(new_n119), .d(new_n117), .out0(new_n120));
  nand02aa1n02x5               g025(.a(new_n120), .b(new_n115), .o1(new_n121));
  ao0012aa1n12x5               g026(.a(new_n111), .b(new_n113), .c(new_n112), .o(new_n122));
  tech160nm_fiao0012aa1n02p5x5 g027(.a(new_n116), .b(new_n118), .c(new_n117), .o(new_n123));
  aoi012aa1n12x5               g028(.a(new_n123), .b(new_n120), .c(new_n122), .o1(new_n124));
  aoai13aa1n12x5               g029(.a(new_n124), .b(new_n121), .c(new_n109), .d(new_n110), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  aoai13aa1n06x5               g031(.a(new_n126), .b(new_n97), .c(new_n125), .d(new_n98), .o1(new_n127));
  aoi112aa1n02x5               g032(.a(new_n126), .b(new_n97), .c(new_n125), .d(new_n98), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n127), .b(new_n128), .out0(\s[10] ));
  norp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  oai012aa1n02x5               g036(.a(new_n131), .b(new_n130), .c(new_n97), .o1(new_n132));
  nor002aa1n06x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanp02aa1n04x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n12x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n127), .c(new_n132), .out0(\s[11] ));
  nor022aa1n08x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(new_n127), .b(new_n132), .o1(new_n138));
  nand02aa1n04x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n137), .b(new_n139), .out0(new_n140));
  aoai13aa1n02x7               g045(.a(new_n140), .b(new_n133), .c(new_n138), .d(new_n134), .o1(new_n141));
  inv000aa1d42x5               g046(.a(new_n135), .o1(new_n142));
  aoai13aa1n03x5               g047(.a(new_n139), .b(new_n142), .c(new_n127), .d(new_n132), .o1(new_n143));
  oai013aa1n02x4               g048(.a(new_n141), .b(new_n143), .c(new_n133), .d(new_n137), .o1(\s[12] ));
  norb02aa1n02x5               g049(.a(new_n98), .b(new_n97), .out0(new_n145));
  nano32aa1n06x5               g050(.a(new_n140), .b(new_n126), .c(new_n145), .d(new_n135), .out0(new_n146));
  nano22aa1n03x5               g051(.a(new_n137), .b(new_n134), .c(new_n139), .out0(new_n147));
  aoi012aa1n02x5               g052(.a(new_n133), .b(\a[10] ), .c(\b[9] ), .o1(new_n148));
  oai112aa1n03x5               g053(.a(new_n147), .b(new_n148), .c(new_n130), .d(new_n97), .o1(new_n149));
  tech160nm_fiaoi012aa1n02p5x5 g054(.a(new_n137), .b(new_n133), .c(new_n139), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n149), .b(new_n150), .o1(new_n151));
  tech160nm_fiaoi012aa1n05x5   g056(.a(new_n151), .b(new_n125), .c(new_n146), .o1(new_n152));
  nor042aa1n04x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(new_n153), .b(new_n154), .out0(new_n155));
  and003aa1n02x5               g060(.a(new_n149), .b(new_n155), .c(new_n150), .o(new_n156));
  aobi12aa1n02x5               g061(.a(new_n156), .b(new_n125), .c(new_n146), .out0(new_n157));
  oab012aa1n02x4               g062(.a(new_n157), .b(new_n152), .c(new_n155), .out0(\s[13] ));
  nor002aa1n12x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  nand22aa1n04x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  norp02aa1n02x5               g066(.a(new_n152), .b(new_n155), .o1(new_n162));
  aoi112aa1n03x5               g067(.a(new_n162), .b(new_n153), .c(new_n160), .d(new_n161), .o1(new_n163));
  oaoi03aa1n02x5               g068(.a(\a[13] ), .b(\b[12] ), .c(new_n152), .o1(new_n164));
  aoi013aa1n02x4               g069(.a(new_n163), .b(new_n164), .c(new_n160), .d(new_n161), .o1(\s[14] ));
  nano23aa1n09x5               g070(.a(new_n153), .b(new_n159), .c(new_n161), .d(new_n154), .out0(new_n166));
  aoai13aa1n06x5               g071(.a(new_n166), .b(new_n151), .c(new_n125), .d(new_n146), .o1(new_n167));
  tech160nm_fiaoi012aa1n04x5   g072(.a(new_n159), .b(new_n153), .c(new_n161), .o1(new_n168));
  nor042aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanp02aa1n04x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n167), .c(new_n168), .out0(\s[15] ));
  nor042aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(new_n167), .b(new_n168), .o1(new_n174));
  nand42aa1n06x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanb02aa1n02x5               g080(.a(new_n173), .b(new_n175), .out0(new_n176));
  aoai13aa1n03x5               g081(.a(new_n176), .b(new_n169), .c(new_n174), .d(new_n171), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n171), .o1(new_n178));
  aoai13aa1n02x5               g083(.a(new_n175), .b(new_n178), .c(new_n167), .d(new_n168), .o1(new_n179));
  oai013aa1n02x4               g084(.a(new_n177), .b(new_n179), .c(new_n169), .d(new_n173), .o1(\s[16] ));
  aoi012aa1n02x5               g085(.a(new_n173), .b(new_n169), .c(new_n175), .o1(new_n181));
  xnrc02aa1n12x5               g086(.a(\b[16] ), .b(\a[17] ), .out0(new_n182));
  nano23aa1n02x4               g087(.a(new_n97), .b(new_n137), .c(new_n139), .d(new_n98), .out0(new_n183));
  nano23aa1n06x5               g088(.a(new_n169), .b(new_n173), .c(new_n175), .d(new_n170), .out0(new_n184));
  nand02aa1n03x5               g089(.a(new_n184), .b(new_n166), .o1(new_n185));
  nano32aa1n03x7               g090(.a(new_n185), .b(new_n183), .c(new_n135), .d(new_n126), .out0(new_n186));
  nanp02aa1n03x5               g091(.a(new_n125), .b(new_n186), .o1(new_n187));
  inv020aa1n03x5               g092(.a(new_n168), .o1(new_n188));
  aobi12aa1n06x5               g093(.a(new_n181), .b(new_n184), .c(new_n188), .out0(new_n189));
  aoai13aa1n12x5               g094(.a(new_n189), .b(new_n185), .c(new_n149), .d(new_n150), .o1(new_n190));
  inv000aa1n02x5               g095(.a(new_n190), .o1(new_n191));
  aoi012aa1n03x5               g096(.a(new_n182), .b(new_n187), .c(new_n191), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(new_n174), .b(new_n184), .o1(new_n193));
  aoi013aa1n02x4               g098(.a(new_n192), .b(new_n193), .c(new_n181), .d(new_n182), .o1(\s[17] ));
  nor042aa1n02x5               g099(.a(\b[16] ), .b(\a[17] ), .o1(new_n195));
  nor042aa1n12x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  nand22aa1n03x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  aoi112aa1n02x5               g103(.a(new_n192), .b(new_n195), .c(new_n197), .d(new_n198), .o1(new_n199));
  oai112aa1n02x7               g104(.a(new_n197), .b(new_n198), .c(new_n192), .d(new_n195), .o1(new_n200));
  norb02aa1n03x4               g105(.a(new_n200), .b(new_n199), .out0(\s[18] ));
  nano22aa1n03x7               g106(.a(new_n182), .b(new_n197), .c(new_n198), .out0(new_n202));
  aoai13aa1n06x5               g107(.a(new_n202), .b(new_n190), .c(new_n125), .d(new_n186), .o1(new_n203));
  aoi012aa1n12x5               g108(.a(new_n196), .b(new_n195), .c(new_n198), .o1(new_n204));
  nor042aa1n12x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nand02aa1d10x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  norb02aa1n15x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n203), .c(new_n204), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n207), .o1(new_n211));
  aoi012aa1n03x5               g116(.a(new_n211), .b(new_n203), .c(new_n204), .o1(new_n212));
  nand02aa1n12x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  norb02aa1n06x4               g118(.a(new_n213), .b(new_n210), .out0(new_n214));
  oabi12aa1n02x5               g119(.a(new_n214), .b(new_n212), .c(new_n205), .out0(new_n215));
  aoai13aa1n02x5               g120(.a(new_n213), .b(new_n211), .c(new_n203), .d(new_n204), .o1(new_n216));
  oai013aa1n02x4               g121(.a(new_n215), .b(new_n216), .c(new_n205), .d(new_n210), .o1(\s[20] ));
  nano23aa1d15x5               g122(.a(new_n205), .b(new_n210), .c(new_n213), .d(new_n206), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n202), .b(new_n219), .out0(new_n220));
  aoai13aa1n06x5               g125(.a(new_n220), .b(new_n190), .c(new_n125), .d(new_n186), .o1(new_n221));
  aoi112aa1n09x5               g126(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n222));
  oai112aa1n06x5               g127(.a(new_n207), .b(new_n214), .c(new_n222), .d(new_n196), .o1(new_n223));
  aoi012aa1n12x5               g128(.a(new_n210), .b(new_n205), .c(new_n213), .o1(new_n224));
  nanp02aa1n02x5               g129(.a(new_n223), .b(new_n224), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  nand02aa1n02x5               g131(.a(new_n221), .b(new_n226), .o1(new_n227));
  xorc02aa1n12x5               g132(.a(\a[21] ), .b(\b[20] ), .out0(new_n228));
  inv040aa1n06x5               g133(.a(new_n204), .o1(new_n229));
  inv030aa1n02x5               g134(.a(new_n224), .o1(new_n230));
  aoi112aa1n02x5               g135(.a(new_n230), .b(new_n228), .c(new_n218), .d(new_n229), .o1(new_n231));
  aoi022aa1n02x5               g136(.a(new_n227), .b(new_n228), .c(new_n221), .d(new_n231), .o1(\s[21] ));
  norp02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  nor042aa1n04x5               g138(.a(\b[21] ), .b(\a[22] ), .o1(new_n234));
  nand42aa1n04x5               g139(.a(\b[21] ), .b(\a[22] ), .o1(new_n235));
  nanb02aa1n02x5               g140(.a(new_n234), .b(new_n235), .out0(new_n236));
  aoai13aa1n03x5               g141(.a(new_n236), .b(new_n233), .c(new_n227), .d(new_n228), .o1(new_n237));
  xnrc02aa1n02x5               g142(.a(\b[20] ), .b(\a[21] ), .out0(new_n238));
  aoai13aa1n02x5               g143(.a(new_n235), .b(new_n238), .c(new_n221), .d(new_n226), .o1(new_n239));
  oai013aa1n02x4               g144(.a(new_n237), .b(new_n239), .c(new_n233), .d(new_n234), .o1(\s[22] ));
  nor002aa1n03x5               g145(.a(new_n238), .b(new_n236), .o1(new_n241));
  and003aa1n02x5               g146(.a(new_n202), .b(new_n241), .c(new_n218), .o(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n190), .c(new_n125), .d(new_n186), .o1(new_n243));
  nanb02aa1n03x5               g148(.a(new_n236), .b(new_n228), .out0(new_n244));
  aoi012aa1n06x5               g149(.a(new_n234), .b(new_n233), .c(new_n235), .o1(new_n245));
  aoai13aa1n12x5               g150(.a(new_n245), .b(new_n244), .c(new_n223), .d(new_n224), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  nand02aa1n02x5               g152(.a(new_n243), .b(new_n247), .o1(new_n248));
  xorc02aa1n12x5               g153(.a(\a[23] ), .b(\b[22] ), .out0(new_n249));
  aoai13aa1n09x5               g154(.a(new_n241), .b(new_n230), .c(new_n218), .d(new_n229), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n249), .o1(new_n251));
  and003aa1n02x5               g156(.a(new_n250), .b(new_n251), .c(new_n245), .o(new_n252));
  aoi022aa1n02x5               g157(.a(new_n248), .b(new_n249), .c(new_n243), .d(new_n252), .o1(\s[23] ));
  norp02aa1n02x5               g158(.a(\b[22] ), .b(\a[23] ), .o1(new_n254));
  nor002aa1n03x5               g159(.a(\b[23] ), .b(\a[24] ), .o1(new_n255));
  tech160nm_finand02aa1n05x5   g160(.a(\b[23] ), .b(\a[24] ), .o1(new_n256));
  nanb02aa1n06x5               g161(.a(new_n255), .b(new_n256), .out0(new_n257));
  aoai13aa1n03x5               g162(.a(new_n257), .b(new_n254), .c(new_n248), .d(new_n249), .o1(new_n258));
  aoai13aa1n02x5               g163(.a(new_n256), .b(new_n251), .c(new_n243), .d(new_n247), .o1(new_n259));
  oai013aa1n03x4               g164(.a(new_n258), .b(new_n259), .c(new_n254), .d(new_n255), .o1(\s[24] ));
  norb02aa1n06x4               g165(.a(new_n249), .b(new_n257), .out0(new_n261));
  inv000aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  nano32aa1n02x4               g167(.a(new_n262), .b(new_n202), .c(new_n241), .d(new_n218), .out0(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n190), .c(new_n125), .d(new_n186), .o1(new_n264));
  aoi012aa1n12x5               g169(.a(new_n255), .b(new_n254), .c(new_n256), .o1(new_n265));
  aoai13aa1n12x5               g170(.a(new_n265), .b(new_n262), .c(new_n250), .d(new_n245), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  nand02aa1n02x5               g172(.a(new_n264), .b(new_n267), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  inv000aa1d42x5               g174(.a(new_n265), .o1(new_n270));
  aoi112aa1n02x5               g175(.a(new_n269), .b(new_n270), .c(new_n246), .d(new_n261), .o1(new_n271));
  aoi022aa1n02x5               g176(.a(new_n268), .b(new_n269), .c(new_n264), .d(new_n271), .o1(\s[25] ));
  nor002aa1n02x5               g177(.a(\b[24] ), .b(\a[25] ), .o1(new_n273));
  norp02aa1n02x5               g178(.a(\b[25] ), .b(\a[26] ), .o1(new_n274));
  nanp02aa1n02x5               g179(.a(\b[25] ), .b(\a[26] ), .o1(new_n275));
  nanb02aa1n02x5               g180(.a(new_n274), .b(new_n275), .out0(new_n276));
  aoai13aa1n02x5               g181(.a(new_n276), .b(new_n273), .c(new_n268), .d(new_n269), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n269), .o1(new_n278));
  aoai13aa1n02x5               g183(.a(new_n275), .b(new_n278), .c(new_n264), .d(new_n267), .o1(new_n279));
  oai013aa1n02x4               g184(.a(new_n277), .b(new_n279), .c(new_n273), .d(new_n274), .o1(\s[26] ));
  norb02aa1n03x5               g185(.a(new_n269), .b(new_n276), .out0(new_n281));
  nona23aa1n02x5               g186(.a(new_n249), .b(new_n228), .c(new_n257), .d(new_n236), .out0(new_n282));
  nano32aa1n03x5               g187(.a(new_n282), .b(new_n281), .c(new_n218), .d(new_n202), .out0(new_n283));
  aoai13aa1n12x5               g188(.a(new_n283), .b(new_n190), .c(new_n125), .d(new_n186), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n281), .b(new_n270), .c(new_n246), .d(new_n261), .o1(new_n285));
  aoi012aa1n02x5               g190(.a(new_n274), .b(new_n273), .c(new_n275), .o1(new_n286));
  nand23aa1n06x5               g191(.a(new_n284), .b(new_n285), .c(new_n286), .o1(new_n287));
  xorc02aa1n12x5               g192(.a(\a[27] ), .b(\b[26] ), .out0(new_n288));
  inv000aa1n02x5               g193(.a(new_n286), .o1(new_n289));
  aoi112aa1n02x5               g194(.a(new_n288), .b(new_n289), .c(new_n266), .d(new_n281), .o1(new_n290));
  aoi022aa1n02x5               g195(.a(new_n287), .b(new_n288), .c(new_n290), .d(new_n284), .o1(\s[27] ));
  nor042aa1n02x5               g196(.a(\b[26] ), .b(\a[27] ), .o1(new_n292));
  tech160nm_finor002aa1n03p5x5 g197(.a(\b[27] ), .b(\a[28] ), .o1(new_n293));
  nanp02aa1n02x5               g198(.a(\b[27] ), .b(\a[28] ), .o1(new_n294));
  nanb02aa1n02x5               g199(.a(new_n293), .b(new_n294), .out0(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n292), .c(new_n287), .d(new_n288), .o1(new_n296));
  oai022aa1n02x5               g201(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n297));
  tech160nm_fiaoi012aa1n05x5   g202(.a(new_n289), .b(new_n266), .c(new_n281), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n288), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n294), .b(new_n299), .c(new_n298), .d(new_n284), .o1(new_n300));
  oai012aa1n03x5               g205(.a(new_n296), .b(new_n300), .c(new_n297), .o1(\s[28] ));
  norb02aa1n03x4               g206(.a(new_n288), .b(new_n295), .out0(new_n302));
  nanp02aa1n03x5               g207(.a(new_n287), .b(new_n302), .o1(new_n303));
  inv000aa1n03x5               g208(.a(new_n302), .o1(new_n304));
  and002aa1n02x5               g209(.a(\b[28] ), .b(\a[29] ), .o(new_n305));
  norp02aa1n02x5               g210(.a(\b[28] ), .b(\a[29] ), .o1(new_n306));
  aoi012aa1n02x5               g211(.a(new_n306), .b(new_n292), .c(new_n294), .o1(new_n307));
  norb03aa1n02x5               g212(.a(new_n307), .b(new_n293), .c(new_n305), .out0(new_n308));
  aoai13aa1n02x5               g213(.a(new_n308), .b(new_n304), .c(new_n298), .d(new_n284), .o1(new_n309));
  aoi012aa1n03x5               g214(.a(new_n293), .b(new_n292), .c(new_n294), .o1(new_n310));
  norp02aa1n02x5               g215(.a(new_n305), .b(new_n306), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n309), .b(new_n311), .c(new_n303), .d(new_n310), .o1(\s[29] ));
  xorb03aa1n02x5               g217(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g218(.a(new_n295), .b(new_n288), .c(new_n311), .out0(new_n314));
  nanp02aa1n03x5               g219(.a(new_n287), .b(new_n314), .o1(new_n315));
  tech160nm_fioaoi03aa1n02p5x5 g220(.a(\a[29] ), .b(\b[28] ), .c(new_n310), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n316), .o1(new_n317));
  norp02aa1n02x5               g222(.a(\b[29] ), .b(\a[30] ), .o1(new_n318));
  nanp02aa1n02x5               g223(.a(\b[29] ), .b(\a[30] ), .o1(new_n319));
  norb02aa1n02x5               g224(.a(new_n319), .b(new_n318), .out0(new_n320));
  inv000aa1n02x5               g225(.a(new_n314), .o1(new_n321));
  nona22aa1n02x4               g226(.a(new_n319), .b(new_n318), .c(new_n306), .out0(new_n322));
  oab012aa1n02x4               g227(.a(new_n322), .b(new_n310), .c(new_n305), .out0(new_n323));
  aoai13aa1n02x7               g228(.a(new_n323), .b(new_n321), .c(new_n298), .d(new_n284), .o1(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n320), .c(new_n315), .d(new_n317), .o1(\s[30] ));
  nano22aa1n02x4               g230(.a(new_n304), .b(new_n311), .c(new_n320), .out0(new_n326));
  nanp02aa1n06x5               g231(.a(new_n287), .b(new_n326), .o1(new_n327));
  xnrc02aa1n02x5               g232(.a(\b[30] ), .b(\a[31] ), .out0(new_n328));
  inv000aa1d42x5               g233(.a(new_n328), .o1(new_n329));
  inv000aa1n02x5               g234(.a(new_n326), .o1(new_n330));
  aoi112aa1n02x5               g235(.a(new_n318), .b(new_n328), .c(new_n316), .d(new_n320), .o1(new_n331));
  aoai13aa1n03x5               g236(.a(new_n331), .b(new_n330), .c(new_n298), .d(new_n284), .o1(new_n332));
  aoi012aa1n02x5               g237(.a(new_n318), .b(new_n316), .c(new_n320), .o1(new_n333));
  aoai13aa1n03x5               g238(.a(new_n332), .b(new_n329), .c(new_n327), .d(new_n333), .o1(\s[31] ));
  xorb03aa1n02x5               g239(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oai012aa1n02x5               g240(.a(new_n107), .b(new_n102), .c(new_n106), .o1(new_n336));
  xnrc02aa1n02x5               g241(.a(new_n336), .b(new_n105), .out0(\s[4] ));
  norb02aa1n02x5               g242(.a(new_n114), .b(new_n113), .out0(new_n338));
  xnbna2aa1n03x5               g243(.a(new_n338), .b(new_n109), .c(new_n110), .out0(\s[5] ));
  nanp02aa1n02x5               g244(.a(new_n109), .b(new_n110), .o1(new_n340));
  aoi012aa1n02x5               g245(.a(new_n113), .b(new_n340), .c(new_n114), .o1(new_n341));
  xnrb03aa1n02x5               g246(.a(new_n341), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g247(.a(new_n119), .b(new_n118), .out0(new_n343));
  inv000aa1d42x5               g248(.a(new_n122), .o1(new_n344));
  aobi12aa1n09x5               g249(.a(new_n115), .b(new_n109), .c(new_n110), .out0(new_n345));
  inv000aa1d42x5               g250(.a(new_n345), .o1(new_n346));
  xnbna2aa1n03x5               g251(.a(new_n343), .b(new_n346), .c(new_n344), .out0(\s[7] ));
  oaoi13aa1n02x5               g252(.a(new_n118), .b(new_n119), .c(new_n345), .d(new_n122), .o1(new_n348));
  xnrb03aa1n02x5               g253(.a(new_n348), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  nanb02aa1n02x5               g254(.a(new_n97), .b(new_n98), .out0(new_n350));
  aoi112aa1n02x5               g255(.a(new_n350), .b(new_n123), .c(new_n120), .d(new_n122), .o1(new_n351));
  aoai13aa1n02x5               g256(.a(new_n351), .b(new_n121), .c(new_n109), .d(new_n110), .o1(new_n352));
  aob012aa1n02x5               g257(.a(new_n352), .b(new_n125), .c(new_n350), .out0(\s[9] ));
endmodule


