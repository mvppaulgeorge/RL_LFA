// Benchmark "adder" written by ABC on Thu Jul 18 09:48:37 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n316,
    new_n318, new_n320, new_n321, new_n322, new_n325, new_n326, new_n328;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nand22aa1n04x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nand42aa1n20x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nor042aa1n04x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nona22aa1n02x4               g006(.a(new_n100), .b(new_n101), .c(new_n99), .out0(new_n102));
  nor002aa1d32x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nano22aa1n02x4               g009(.a(new_n103), .b(new_n100), .c(new_n104), .out0(new_n105));
  inv000aa1d42x5               g010(.a(\a[4] ), .o1(new_n106));
  inv000aa1d42x5               g011(.a(\b[3] ), .o1(new_n107));
  aoi012aa1n12x5               g012(.a(new_n103), .b(new_n106), .c(new_n107), .o1(new_n108));
  aobi12aa1n06x5               g013(.a(new_n108), .b(new_n105), .c(new_n102), .out0(new_n109));
  nanp02aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nor022aa1n16x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor002aa1n16x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nano23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n12x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  inv000aa1d42x5               g020(.a(\b[4] ), .o1(new_n116));
  nanb02aa1d24x5               g021(.a(\a[5] ), .b(new_n116), .out0(new_n117));
  nand42aa1n03x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  oai112aa1n06x5               g023(.a(new_n117), .b(new_n118), .c(new_n107), .d(new_n106), .o1(new_n119));
  nona22aa1n02x4               g024(.a(new_n114), .b(new_n119), .c(new_n115), .out0(new_n120));
  aoi022aa1d24x5               g025(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n121));
  oaih22aa1d12x5               g026(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  nand22aa1n04x5               g027(.a(new_n122), .b(new_n121), .o1(new_n123));
  nor002aa1n03x5               g028(.a(new_n113), .b(new_n111), .o1(new_n124));
  aoi022aa1d24x5               g029(.a(new_n123), .b(new_n124), .c(\b[7] ), .d(\a[8] ), .o1(new_n125));
  inv000aa1d42x5               g030(.a(new_n125), .o1(new_n126));
  tech160nm_fioai012aa1n03p5x5 g031(.a(new_n126), .b(new_n120), .c(new_n109), .o1(new_n127));
  tech160nm_fioaoi03aa1n03p5x5 g032(.a(new_n97), .b(new_n98), .c(new_n127), .o1(new_n128));
  xorc02aa1n12x5               g033(.a(\a[10] ), .b(\b[9] ), .out0(new_n129));
  xnrc02aa1n02x5               g034(.a(new_n128), .b(new_n129), .out0(\s[10] ));
  and002aa1n12x5               g035(.a(\b[9] ), .b(\a[10] ), .o(new_n131));
  inv040aa1n02x5               g036(.a(new_n131), .o1(new_n132));
  nanp02aa1n03x5               g037(.a(new_n128), .b(new_n129), .o1(new_n133));
  nor002aa1d32x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n04x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n133), .c(new_n132), .out0(\s[11] ));
  inv030aa1n06x5               g042(.a(new_n134), .o1(new_n138));
  nona22aa1n03x5               g043(.a(new_n133), .b(new_n136), .c(new_n131), .out0(new_n139));
  nor042aa1n06x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n139), .c(new_n138), .out0(\s[12] ));
  norb03aa1n12x5               g048(.a(new_n100), .b(new_n99), .c(new_n101), .out0(new_n144));
  nanb03aa1n12x5               g049(.a(new_n103), .b(new_n104), .c(new_n100), .out0(new_n145));
  oai012aa1n12x5               g050(.a(new_n108), .b(new_n144), .c(new_n145), .o1(new_n146));
  nona23aa1n03x5               g051(.a(new_n112), .b(new_n110), .c(new_n113), .d(new_n111), .out0(new_n147));
  nor043aa1n03x5               g052(.a(new_n147), .b(new_n115), .c(new_n119), .o1(new_n148));
  xnrc02aa1n02x5               g053(.a(\b[8] ), .b(\a[9] ), .out0(new_n149));
  nona23aa1n09x5               g054(.a(new_n141), .b(new_n135), .c(new_n134), .d(new_n140), .out0(new_n150));
  norb03aa1n03x5               g055(.a(new_n129), .b(new_n150), .c(new_n149), .out0(new_n151));
  aoai13aa1n06x5               g056(.a(new_n151), .b(new_n125), .c(new_n148), .d(new_n146), .o1(new_n152));
  nano23aa1n06x5               g057(.a(new_n134), .b(new_n140), .c(new_n141), .d(new_n135), .out0(new_n153));
  oaih22aa1n04x5               g058(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n154));
  oaoi03aa1n12x5               g059(.a(\a[12] ), .b(\b[11] ), .c(new_n138), .o1(new_n155));
  aoi013aa1n02x4               g060(.a(new_n155), .b(new_n153), .c(new_n154), .d(new_n132), .o1(new_n156));
  xnrc02aa1n12x5               g061(.a(\b[12] ), .b(\a[13] ), .out0(new_n157));
  xobna2aa1n03x5               g062(.a(new_n157), .b(new_n152), .c(new_n156), .out0(\s[13] ));
  orn002aa1n02x5               g063(.a(\a[13] ), .b(\b[12] ), .o(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n157), .c(new_n152), .d(new_n156), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  tech160nm_fixnrc02aa1n04x5   g066(.a(\b[13] ), .b(\a[14] ), .out0(new_n162));
  aoi112aa1n03x4               g067(.a(new_n162), .b(new_n157), .c(new_n152), .d(new_n156), .o1(new_n163));
  oaoi03aa1n02x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n159), .o1(new_n164));
  norp02aa1n02x5               g069(.a(new_n163), .b(new_n164), .o1(new_n165));
  nor002aa1d32x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  inv040aa1n09x5               g071(.a(new_n166), .o1(new_n167));
  nand02aa1n06x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  xnbna2aa1n03x5               g073(.a(new_n165), .b(new_n168), .c(new_n167), .out0(\s[15] ));
  norb02aa1n02x5               g074(.a(new_n168), .b(new_n166), .out0(new_n170));
  oai012aa1n02x5               g075(.a(new_n170), .b(new_n163), .c(new_n164), .o1(new_n171));
  nor002aa1d32x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nand02aa1n04x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  norb02aa1n12x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  aobi12aa1n02x5               g079(.a(new_n174), .b(new_n171), .c(new_n167), .out0(new_n175));
  nona22aa1n02x4               g080(.a(new_n171), .b(new_n174), .c(new_n166), .out0(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(\s[16] ));
  aoi012aa1n09x5               g082(.a(new_n125), .b(new_n148), .c(new_n146), .o1(new_n178));
  nona23aa1d18x5               g083(.a(new_n173), .b(new_n168), .c(new_n166), .d(new_n172), .out0(new_n179));
  nor043aa1n03x5               g084(.a(new_n179), .b(new_n162), .c(new_n157), .o1(new_n180));
  nand02aa1n02x5               g085(.a(new_n151), .b(new_n180), .o1(new_n181));
  nano22aa1n02x5               g086(.a(new_n150), .b(new_n132), .c(new_n154), .out0(new_n182));
  oaih22aa1d12x5               g087(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n183));
  aob012aa1n12x5               g088(.a(new_n183), .b(\b[13] ), .c(\a[14] ), .out0(new_n184));
  oaoi03aa1n09x5               g089(.a(\a[16] ), .b(\b[15] ), .c(new_n167), .o1(new_n185));
  oabi12aa1n18x5               g090(.a(new_n185), .b(new_n179), .c(new_n184), .out0(new_n186));
  oaoi13aa1n09x5               g091(.a(new_n186), .b(new_n180), .c(new_n182), .d(new_n155), .o1(new_n187));
  oai012aa1n12x5               g092(.a(new_n187), .b(new_n178), .c(new_n181), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor002aa1d32x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nand42aa1d28x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  norb02aa1n06x4               g096(.a(new_n191), .b(new_n190), .out0(new_n192));
  inv040aa1d28x5               g097(.a(\a[17] ), .o1(new_n193));
  inv040aa1d32x5               g098(.a(\b[16] ), .o1(new_n194));
  oaoi03aa1n02x5               g099(.a(new_n193), .b(new_n194), .c(new_n188), .o1(new_n195));
  xnrc02aa1n02x5               g100(.a(new_n195), .b(new_n192), .out0(\s[18] ));
  tech160nm_fixorc02aa1n02p5x5 g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  nand23aa1n03x5               g102(.a(new_n188), .b(new_n197), .c(new_n192), .o1(new_n198));
  aoai13aa1n12x5               g103(.a(new_n191), .b(new_n190), .c(new_n193), .d(new_n194), .o1(new_n199));
  nor002aa1n16x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nand02aa1n03x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  xobna2aa1n03x5               g107(.a(new_n202), .b(new_n198), .c(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g109(.a(new_n199), .o1(new_n205));
  aoi013aa1n02x4               g110(.a(new_n205), .b(new_n188), .c(new_n197), .d(new_n192), .o1(new_n206));
  inv000aa1d42x5               g111(.a(new_n200), .o1(new_n207));
  norp02aa1n04x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nanp02aa1n04x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nanb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  oaoi13aa1n02x5               g115(.a(new_n210), .b(new_n207), .c(new_n206), .d(new_n202), .o1(new_n211));
  aoi012aa1n02x5               g116(.a(new_n202), .b(new_n198), .c(new_n199), .o1(new_n212));
  nano22aa1n03x5               g117(.a(new_n212), .b(new_n207), .c(new_n210), .out0(new_n213));
  norp02aa1n02x5               g118(.a(new_n211), .b(new_n213), .o1(\s[20] ));
  nona23aa1n09x5               g119(.a(new_n209), .b(new_n201), .c(new_n200), .d(new_n208), .out0(new_n215));
  nano22aa1n03x7               g120(.a(new_n215), .b(new_n197), .c(new_n192), .out0(new_n216));
  oai012aa1n02x5               g121(.a(new_n209), .b(new_n208), .c(new_n200), .o1(new_n217));
  oai012aa1n12x5               g122(.a(new_n217), .b(new_n215), .c(new_n199), .o1(new_n218));
  tech160nm_fixorc02aa1n04x5   g123(.a(\a[21] ), .b(\b[20] ), .out0(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n218), .c(new_n188), .d(new_n216), .o1(new_n220));
  aoi112aa1n02x5               g125(.a(new_n219), .b(new_n218), .c(new_n188), .d(new_n216), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n220), .b(new_n221), .out0(\s[21] ));
  inv000aa1d42x5               g127(.a(\a[21] ), .o1(new_n223));
  nanb02aa1n02x5               g128(.a(\b[20] ), .b(new_n223), .out0(new_n224));
  tech160nm_fixorc02aa1n04x5   g129(.a(\a[22] ), .b(\b[21] ), .out0(new_n225));
  xnbna2aa1n03x5               g130(.a(new_n225), .b(new_n220), .c(new_n224), .out0(\s[22] ));
  and002aa1n09x5               g131(.a(new_n225), .b(new_n219), .o(new_n227));
  oaoi03aa1n02x5               g132(.a(\a[22] ), .b(\b[21] ), .c(new_n224), .o1(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n218), .c(new_n227), .o1(new_n229));
  nand23aa1n03x5               g134(.a(new_n188), .b(new_n216), .c(new_n227), .o1(new_n230));
  tech160nm_fixnrc02aa1n04x5   g135(.a(\b[22] ), .b(\a[23] ), .out0(new_n231));
  xobna2aa1n03x5               g136(.a(new_n231), .b(new_n230), .c(new_n229), .out0(\s[23] ));
  inv000aa1n02x5               g137(.a(new_n229), .o1(new_n233));
  aoi013aa1n02x4               g138(.a(new_n233), .b(new_n188), .c(new_n216), .d(new_n227), .o1(new_n234));
  orn002aa1n24x5               g139(.a(\a[23] ), .b(\b[22] ), .o(new_n235));
  xnrc02aa1n02x5               g140(.a(\b[23] ), .b(\a[24] ), .out0(new_n236));
  oaoi13aa1n02x5               g141(.a(new_n236), .b(new_n235), .c(new_n234), .d(new_n231), .o1(new_n237));
  tech160nm_fiaoi012aa1n02p5x5 g142(.a(new_n231), .b(new_n230), .c(new_n229), .o1(new_n238));
  nano22aa1n03x5               g143(.a(new_n238), .b(new_n235), .c(new_n236), .out0(new_n239));
  norp02aa1n02x5               g144(.a(new_n237), .b(new_n239), .o1(\s[24] ));
  nor042aa1n02x5               g145(.a(new_n236), .b(new_n231), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n228), .c(new_n218), .d(new_n227), .o1(new_n242));
  oaoi03aa1n02x5               g147(.a(\a[24] ), .b(\b[23] ), .c(new_n235), .o1(new_n243));
  inv000aa1n03x5               g148(.a(new_n243), .o1(new_n244));
  nanp02aa1n06x5               g149(.a(new_n242), .b(new_n244), .o1(new_n245));
  inv000aa1n02x5               g150(.a(new_n245), .o1(new_n246));
  nano23aa1n02x4               g151(.a(new_n236), .b(new_n231), .c(new_n225), .d(new_n219), .out0(new_n247));
  nanp03aa1n03x5               g152(.a(new_n188), .b(new_n216), .c(new_n247), .o1(new_n248));
  xorc02aa1n12x5               g153(.a(\a[25] ), .b(\b[24] ), .out0(new_n249));
  xnbna2aa1n03x5               g154(.a(new_n249), .b(new_n248), .c(new_n246), .out0(\s[25] ));
  aoi013aa1n02x5               g155(.a(new_n245), .b(new_n188), .c(new_n216), .d(new_n247), .o1(new_n251));
  orn002aa1n02x5               g156(.a(\a[25] ), .b(\b[24] ), .o(new_n252));
  inv000aa1d42x5               g157(.a(new_n249), .o1(new_n253));
  xnrc02aa1n02x5               g158(.a(\b[25] ), .b(\a[26] ), .out0(new_n254));
  oaoi13aa1n02x7               g159(.a(new_n254), .b(new_n252), .c(new_n251), .d(new_n253), .o1(new_n255));
  aoi012aa1n03x5               g160(.a(new_n253), .b(new_n248), .c(new_n246), .o1(new_n256));
  nano22aa1n02x4               g161(.a(new_n256), .b(new_n252), .c(new_n254), .out0(new_n257));
  norp02aa1n03x5               g162(.a(new_n255), .b(new_n257), .o1(\s[26] ));
  inv000aa1d42x5               g163(.a(\a[27] ), .o1(new_n259));
  oaoi03aa1n12x5               g164(.a(\a[26] ), .b(\b[25] ), .c(new_n252), .o1(new_n260));
  norb02aa1n06x4               g165(.a(new_n249), .b(new_n254), .out0(new_n261));
  inv000aa1n03x5               g166(.a(new_n261), .o1(new_n262));
  aoi012aa1n06x5               g167(.a(new_n262), .b(new_n242), .c(new_n244), .o1(new_n263));
  nano32aa1n03x7               g168(.a(new_n262), .b(new_n216), .c(new_n227), .d(new_n241), .out0(new_n264));
  aoi112aa1n06x5               g169(.a(new_n263), .b(new_n260), .c(new_n188), .d(new_n264), .o1(new_n265));
  xorb03aa1n03x5               g170(.a(new_n265), .b(\b[26] ), .c(new_n259), .out0(\s[27] ));
  nor042aa1n04x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  inv000aa1n03x5               g172(.a(new_n267), .o1(new_n268));
  tech160nm_fixorc02aa1n04x5   g173(.a(\a[27] ), .b(\b[26] ), .out0(new_n269));
  inv000aa1d42x5               g174(.a(new_n269), .o1(new_n270));
  tech160nm_fixorc02aa1n04x5   g175(.a(\a[28] ), .b(\b[27] ), .out0(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  oaoi13aa1n02x7               g177(.a(new_n272), .b(new_n268), .c(new_n265), .d(new_n270), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n260), .o1(new_n274));
  nand02aa1d06x5               g179(.a(new_n245), .b(new_n261), .o1(new_n275));
  nanb03aa1n02x5               g180(.a(new_n149), .b(new_n153), .c(new_n129), .out0(new_n276));
  nona23aa1n03x5               g181(.a(new_n170), .b(new_n174), .c(new_n162), .d(new_n157), .out0(new_n277));
  norp02aa1n02x5               g182(.a(new_n277), .b(new_n276), .o1(new_n278));
  nanp03aa1n02x5               g183(.a(new_n153), .b(new_n132), .c(new_n154), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n155), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n186), .o1(new_n281));
  aoai13aa1n02x5               g186(.a(new_n281), .b(new_n277), .c(new_n279), .d(new_n280), .o1(new_n282));
  aoai13aa1n06x5               g187(.a(new_n264), .b(new_n282), .c(new_n127), .d(new_n278), .o1(new_n283));
  nanp03aa1n06x5               g188(.a(new_n275), .b(new_n283), .c(new_n274), .o1(new_n284));
  aoi112aa1n03x4               g189(.a(new_n271), .b(new_n267), .c(new_n284), .d(new_n269), .o1(new_n285));
  norp02aa1n03x5               g190(.a(new_n273), .b(new_n285), .o1(\s[28] ));
  tech160nm_fixorc02aa1n05x5   g191(.a(\a[29] ), .b(\b[28] ), .out0(new_n287));
  inv000aa1d42x5               g192(.a(new_n287), .o1(new_n288));
  inv000aa1d42x5               g193(.a(\a[28] ), .o1(new_n289));
  xroi22aa1d04x5               g194(.a(new_n259), .b(\b[26] ), .c(new_n289), .d(\b[27] ), .out0(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  tech160nm_fioaoi03aa1n03p5x5 g196(.a(\a[28] ), .b(\b[27] ), .c(new_n268), .o1(new_n292));
  inv000aa1n03x5               g197(.a(new_n292), .o1(new_n293));
  oaoi13aa1n02x7               g198(.a(new_n288), .b(new_n293), .c(new_n265), .d(new_n291), .o1(new_n294));
  aoi112aa1n03x4               g199(.a(new_n287), .b(new_n292), .c(new_n284), .d(new_n290), .o1(new_n295));
  norp02aa1n03x5               g200(.a(new_n294), .b(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g201(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  tech160nm_fixorc02aa1n04x5   g202(.a(\a[30] ), .b(\b[29] ), .out0(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  nano22aa1n02x4               g204(.a(new_n288), .b(new_n269), .c(new_n271), .out0(new_n300));
  inv000aa1n02x5               g205(.a(new_n300), .o1(new_n301));
  oao003aa1n03x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n302));
  oaoi13aa1n03x5               g207(.a(new_n299), .b(new_n302), .c(new_n265), .d(new_n301), .o1(new_n303));
  aoi013aa1n03x5               g208(.a(new_n301), .b(new_n275), .c(new_n283), .d(new_n274), .o1(new_n304));
  nano22aa1n03x5               g209(.a(new_n304), .b(new_n299), .c(new_n302), .out0(new_n305));
  norp02aa1n03x5               g210(.a(new_n303), .b(new_n305), .o1(\s[30] ));
  nano32aa1n02x4               g211(.a(new_n270), .b(new_n298), .c(new_n271), .d(new_n287), .out0(new_n307));
  inv000aa1n02x5               g212(.a(new_n307), .o1(new_n308));
  aoi013aa1n03x5               g213(.a(new_n308), .b(new_n275), .c(new_n283), .d(new_n274), .o1(new_n309));
  tech160nm_fioaoi03aa1n03p5x5 g214(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n310), .o1(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  nano22aa1n03x5               g217(.a(new_n309), .b(new_n311), .c(new_n312), .out0(new_n313));
  oaoi13aa1n03x5               g218(.a(new_n312), .b(new_n311), .c(new_n265), .d(new_n308), .o1(new_n314));
  norp02aa1n03x5               g219(.a(new_n314), .b(new_n313), .o1(\s[31] ));
  norb02aa1n02x5               g220(.a(new_n104), .b(new_n103), .out0(new_n316));
  xobna2aa1n03x5               g221(.a(new_n316), .b(new_n102), .c(new_n100), .out0(\s[3] ));
  oabi12aa1n02x5               g222(.a(new_n103), .b(new_n144), .c(new_n145), .out0(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorc02aa1n02x5               g224(.a(\a[5] ), .b(\b[4] ), .out0(new_n320));
  oaoi13aa1n02x5               g225(.a(new_n320), .b(new_n146), .c(new_n106), .d(new_n107), .o1(new_n321));
  oai112aa1n02x5               g226(.a(new_n146), .b(new_n320), .c(new_n107), .d(new_n106), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n322), .b(new_n321), .out0(\s[5] ));
  xobna2aa1n03x5               g228(.a(new_n115), .b(new_n322), .c(new_n117), .out0(\s[6] ));
  nanb03aa1n02x5               g229(.a(new_n115), .b(new_n322), .c(new_n117), .out0(new_n325));
  aob012aa1n02x5               g230(.a(new_n325), .b(\b[5] ), .c(\a[6] ), .out0(new_n326));
  xnrb03aa1n02x5               g231(.a(new_n326), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g232(.a(\a[7] ), .b(\b[6] ), .c(new_n326), .o1(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g234(.a(new_n178), .b(\b[8] ), .c(new_n97), .out0(\s[9] ));
endmodule


