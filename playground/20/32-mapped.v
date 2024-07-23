// Benchmark "adder" written by ABC on Wed Jul 17 22:29:42 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n313, new_n315, new_n318, new_n320,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nand22aa1n12x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nand02aa1n06x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nor042aa1n09x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand42aa1n02x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  oai112aa1n06x5               g006(.a(new_n101), .b(new_n99), .c(new_n100), .d(new_n98), .o1(new_n102));
  oa0022aa1n06x5               g007(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n103));
  aoi022aa1n09x5               g008(.a(new_n102), .b(new_n103), .c(\b[3] ), .d(\a[4] ), .o1(new_n104));
  nor042aa1n02x5               g009(.a(\b[7] ), .b(\a[8] ), .o1(new_n105));
  nor022aa1n04x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  nand22aa1n04x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nor022aa1n08x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nona23aa1n02x4               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  nand22aa1n04x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor042aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand22aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanb03aa1n02x5               g018(.a(new_n112), .b(new_n113), .c(new_n111), .out0(new_n114));
  nor043aa1n03x5               g019(.a(new_n110), .b(new_n114), .c(new_n105), .o1(new_n115));
  nano22aa1n03x7               g020(.a(new_n112), .b(new_n111), .c(new_n113), .out0(new_n116));
  aoi112aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n117));
  oai022aa1n09x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  aoi113aa1n09x5               g023(.a(new_n117), .b(new_n105), .c(new_n116), .d(new_n107), .e(new_n118), .o1(new_n119));
  aob012aa1n09x5               g024(.a(new_n119), .b(new_n115), .c(new_n104), .out0(new_n120));
  tech160nm_fixorc02aa1n02p5x5 g025(.a(\a[9] ), .b(\b[8] ), .out0(new_n121));
  aoi012aa1n02x5               g026(.a(new_n97), .b(new_n120), .c(new_n121), .o1(new_n122));
  xnrb03aa1n02x5               g027(.a(new_n122), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand42aa1d28x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nor042aa1d18x5               g029(.a(\b[10] ), .b(\a[11] ), .o1(new_n125));
  nand02aa1d06x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  nanb02aa1n02x5               g031(.a(new_n125), .b(new_n126), .out0(new_n127));
  nor042aa1n09x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  aoi012aa1n02x5               g033(.a(new_n105), .b(\a[6] ), .c(\b[5] ), .o1(new_n129));
  nanp03aa1n02x5               g034(.a(new_n116), .b(new_n118), .c(new_n129), .o1(new_n130));
  nona22aa1n03x5               g035(.a(new_n130), .b(new_n117), .c(new_n105), .out0(new_n131));
  aoai13aa1n02x5               g036(.a(new_n121), .b(new_n131), .c(new_n104), .d(new_n115), .o1(new_n132));
  nona22aa1n03x5               g037(.a(new_n132), .b(new_n97), .c(new_n128), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n127), .b(new_n133), .c(new_n124), .out0(\s[11] ));
  aoi013aa1n03x5               g039(.a(new_n125), .b(new_n133), .c(new_n126), .d(new_n124), .o1(new_n135));
  xnrb03aa1n03x5               g040(.a(new_n135), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  inv000aa1n02x5               g041(.a(new_n124), .o1(new_n137));
  nona32aa1n03x5               g042(.a(new_n121), .b(new_n137), .c(new_n125), .d(new_n128), .out0(new_n138));
  norp02aa1n04x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand02aa1d04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanb03aa1n12x5               g045(.a(new_n139), .b(new_n140), .c(new_n126), .out0(new_n141));
  norp02aa1n02x5               g046(.a(new_n138), .b(new_n141), .o1(new_n142));
  aoai13aa1n06x5               g047(.a(new_n142), .b(new_n131), .c(new_n104), .d(new_n115), .o1(new_n143));
  inv040aa1n03x5               g048(.a(new_n125), .o1(new_n144));
  oai112aa1n06x5               g049(.a(new_n144), .b(new_n124), .c(new_n97), .d(new_n128), .o1(new_n145));
  oaoi03aa1n03x5               g050(.a(\a[12] ), .b(\b[11] ), .c(new_n144), .o1(new_n146));
  oabi12aa1n18x5               g051(.a(new_n146), .b(new_n145), .c(new_n141), .out0(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  nor002aa1d32x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nand42aa1d28x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nanb02aa1n02x5               g055(.a(new_n149), .b(new_n150), .out0(new_n151));
  xobna2aa1n03x5               g056(.a(new_n151), .b(new_n143), .c(new_n148), .out0(\s[13] ));
  inv000aa1n06x5               g057(.a(new_n149), .o1(new_n153));
  aoai13aa1n02x5               g058(.a(new_n153), .b(new_n151), .c(new_n143), .d(new_n148), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nand42aa1n08x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nano23aa1d15x5               g062(.a(new_n149), .b(new_n156), .c(new_n157), .d(new_n150), .out0(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  tech160nm_fioaoi03aa1n04x5   g064(.a(\a[14] ), .b(\b[13] ), .c(new_n153), .o1(new_n160));
  inv000aa1n02x5               g065(.a(new_n160), .o1(new_n161));
  aoai13aa1n06x5               g066(.a(new_n161), .b(new_n159), .c(new_n143), .d(new_n148), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nand02aa1n03x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nor002aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nand02aa1n03x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  aoi112aa1n02x5               g073(.a(new_n164), .b(new_n168), .c(new_n162), .d(new_n165), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n168), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n170));
  norb02aa1n03x4               g075(.a(new_n170), .b(new_n169), .out0(\s[16] ));
  nano22aa1n03x7               g076(.a(new_n139), .b(new_n126), .c(new_n140), .out0(new_n172));
  nona23aa1d18x5               g077(.a(new_n167), .b(new_n165), .c(new_n164), .d(new_n166), .out0(new_n173));
  nano23aa1d15x5               g078(.a(new_n138), .b(new_n173), .c(new_n158), .d(new_n172), .out0(new_n174));
  aoai13aa1n06x5               g079(.a(new_n174), .b(new_n131), .c(new_n104), .d(new_n115), .o1(new_n175));
  inv000aa1d42x5               g080(.a(new_n173), .o1(new_n176));
  aoai13aa1n04x5               g081(.a(new_n176), .b(new_n160), .c(new_n147), .d(new_n158), .o1(new_n177));
  aoi012aa1n02x7               g082(.a(new_n166), .b(new_n164), .c(new_n167), .o1(new_n178));
  nand23aa1n06x5               g083(.a(new_n175), .b(new_n177), .c(new_n178), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g085(.a(\a[18] ), .o1(new_n181));
  inv040aa1d30x5               g086(.a(\a[17] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[16] ), .o1(new_n183));
  oaoi03aa1n03x5               g088(.a(new_n182), .b(new_n183), .c(new_n179), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(new_n181), .out0(\s[18] ));
  oai012aa1n02x5               g090(.a(new_n124), .b(\b[10] ), .c(\a[11] ), .o1(new_n186));
  oab012aa1n02x4               g091(.a(new_n186), .b(new_n128), .c(new_n97), .out0(new_n187));
  aoai13aa1n03x5               g092(.a(new_n158), .b(new_n146), .c(new_n187), .d(new_n172), .o1(new_n188));
  aoai13aa1n06x5               g093(.a(new_n178), .b(new_n173), .c(new_n188), .d(new_n161), .o1(new_n189));
  xroi22aa1d06x4               g094(.a(new_n182), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n189), .c(new_n120), .d(new_n174), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(\b[17] ), .b(\a[18] ), .o1(new_n192));
  oai022aa1n12x5               g097(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n193));
  and002aa1n02x5               g098(.a(new_n193), .b(new_n192), .o(new_n194));
  inv000aa1d42x5               g099(.a(new_n194), .o1(new_n195));
  nor042aa1n06x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanp02aa1n04x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n191), .c(new_n195), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g105(.a(new_n196), .o1(new_n201));
  aobi12aa1n02x5               g106(.a(new_n198), .b(new_n191), .c(new_n195), .out0(new_n202));
  nor042aa1n04x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nand22aa1n04x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nanb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  nano22aa1n02x4               g110(.a(new_n202), .b(new_n201), .c(new_n205), .out0(new_n206));
  aoai13aa1n03x5               g111(.a(new_n198), .b(new_n194), .c(new_n179), .d(new_n190), .o1(new_n207));
  tech160nm_fiaoi012aa1n02p5x5 g112(.a(new_n205), .b(new_n207), .c(new_n201), .o1(new_n208));
  norp02aa1n03x5               g113(.a(new_n208), .b(new_n206), .o1(\s[20] ));
  nanb03aa1n06x5               g114(.a(new_n203), .b(new_n204), .c(new_n197), .out0(new_n210));
  nona22aa1n09x5               g115(.a(new_n190), .b(new_n196), .c(new_n210), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n189), .c(new_n120), .d(new_n174), .o1(new_n213));
  oai112aa1n03x5               g118(.a(new_n193), .b(new_n192), .c(\b[18] ), .d(\a[19] ), .o1(new_n214));
  aoi012aa1n06x5               g119(.a(new_n203), .b(new_n196), .c(new_n204), .o1(new_n215));
  oai012aa1n04x7               g120(.a(new_n215), .b(new_n214), .c(new_n210), .o1(new_n216));
  xnrc02aa1n12x5               g121(.a(\b[20] ), .b(\a[21] ), .out0(new_n217));
  aoib12aa1n06x5               g122(.a(new_n217), .b(new_n213), .c(new_n216), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n217), .o1(new_n219));
  aoi112aa1n03x4               g124(.a(new_n219), .b(new_n216), .c(new_n179), .d(new_n212), .o1(new_n220));
  norp02aa1n02x5               g125(.a(new_n218), .b(new_n220), .o1(\s[21] ));
  nor042aa1n04x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  inv040aa1n03x5               g127(.a(new_n222), .o1(new_n223));
  xnrc02aa1n12x5               g128(.a(\b[21] ), .b(\a[22] ), .out0(new_n224));
  nano22aa1n02x4               g129(.a(new_n218), .b(new_n223), .c(new_n224), .out0(new_n225));
  aoai13aa1n03x5               g130(.a(new_n219), .b(new_n216), .c(new_n179), .d(new_n212), .o1(new_n226));
  aoi012aa1n03x5               g131(.a(new_n224), .b(new_n226), .c(new_n223), .o1(new_n227));
  norp02aa1n03x5               g132(.a(new_n227), .b(new_n225), .o1(\s[22] ));
  nor042aa1n03x5               g133(.a(new_n224), .b(new_n217), .o1(new_n229));
  nano32aa1n02x4               g134(.a(new_n210), .b(new_n190), .c(new_n229), .d(new_n201), .out0(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n189), .c(new_n120), .d(new_n174), .o1(new_n231));
  oao003aa1n02x5               g136(.a(\a[22] ), .b(\b[21] ), .c(new_n223), .carry(new_n232));
  inv000aa1n02x5               g137(.a(new_n232), .o1(new_n233));
  tech160nm_fiaoi012aa1n03p5x5 g138(.a(new_n233), .b(new_n216), .c(new_n229), .o1(new_n234));
  xnrc02aa1n12x5               g139(.a(\b[22] ), .b(\a[23] ), .out0(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  xnbna2aa1n03x5               g141(.a(new_n236), .b(new_n231), .c(new_n234), .out0(\s[23] ));
  nor042aa1n03x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  tech160nm_fiaoi012aa1n02p5x5 g144(.a(new_n235), .b(new_n231), .c(new_n234), .o1(new_n240));
  xnrc02aa1n02x5               g145(.a(\b[23] ), .b(\a[24] ), .out0(new_n241));
  nano22aa1n02x4               g146(.a(new_n240), .b(new_n239), .c(new_n241), .out0(new_n242));
  inv000aa1n02x5               g147(.a(new_n234), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n236), .b(new_n243), .c(new_n179), .d(new_n230), .o1(new_n244));
  aoi012aa1n03x5               g149(.a(new_n241), .b(new_n244), .c(new_n239), .o1(new_n245));
  norp02aa1n03x5               g150(.a(new_n245), .b(new_n242), .o1(\s[24] ));
  nor002aa1n02x5               g151(.a(new_n241), .b(new_n235), .o1(new_n247));
  nano22aa1n03x7               g152(.a(new_n211), .b(new_n229), .c(new_n247), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n189), .c(new_n120), .d(new_n174), .o1(new_n249));
  nano22aa1n02x4               g154(.a(new_n203), .b(new_n197), .c(new_n204), .out0(new_n250));
  oai012aa1n02x5               g155(.a(new_n192), .b(\b[18] ), .c(\a[19] ), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n193), .b(new_n251), .out0(new_n252));
  inv040aa1n03x5               g157(.a(new_n215), .o1(new_n253));
  aoai13aa1n04x5               g158(.a(new_n229), .b(new_n253), .c(new_n252), .d(new_n250), .o1(new_n254));
  inv000aa1n02x5               g159(.a(new_n247), .o1(new_n255));
  oao003aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .c(new_n239), .carry(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n255), .c(new_n254), .d(new_n232), .o1(new_n257));
  xnrc02aa1n12x5               g162(.a(\b[24] ), .b(\a[25] ), .out0(new_n258));
  aoib12aa1n06x5               g163(.a(new_n258), .b(new_n249), .c(new_n257), .out0(new_n259));
  inv000aa1d42x5               g164(.a(new_n258), .o1(new_n260));
  aoi112aa1n03x4               g165(.a(new_n260), .b(new_n257), .c(new_n179), .d(new_n248), .o1(new_n261));
  norp02aa1n02x5               g166(.a(new_n259), .b(new_n261), .o1(\s[25] ));
  nor042aa1n03x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  xnrc02aa1n02x5               g169(.a(\b[25] ), .b(\a[26] ), .out0(new_n265));
  nano22aa1n02x4               g170(.a(new_n259), .b(new_n264), .c(new_n265), .out0(new_n266));
  aoai13aa1n03x5               g171(.a(new_n260), .b(new_n257), .c(new_n179), .d(new_n248), .o1(new_n267));
  aoi012aa1n03x5               g172(.a(new_n265), .b(new_n267), .c(new_n264), .o1(new_n268));
  nor002aa1n02x5               g173(.a(new_n268), .b(new_n266), .o1(\s[26] ));
  nor042aa1n04x5               g174(.a(new_n265), .b(new_n258), .o1(new_n270));
  nano32aa1n03x7               g175(.a(new_n211), .b(new_n270), .c(new_n229), .d(new_n247), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n189), .c(new_n120), .d(new_n174), .o1(new_n272));
  oao003aa1n03x5               g177(.a(\a[26] ), .b(\b[25] ), .c(new_n264), .carry(new_n273));
  aobi12aa1n06x5               g178(.a(new_n273), .b(new_n257), .c(new_n270), .out0(new_n274));
  xorc02aa1n12x5               g179(.a(\a[27] ), .b(\b[26] ), .out0(new_n275));
  xnbna2aa1n03x5               g180(.a(new_n275), .b(new_n274), .c(new_n272), .out0(\s[27] ));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  inv040aa1n03x5               g182(.a(new_n277), .o1(new_n278));
  aobi12aa1n02x7               g183(.a(new_n275), .b(new_n274), .c(new_n272), .out0(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  nano22aa1n03x5               g185(.a(new_n279), .b(new_n278), .c(new_n280), .out0(new_n281));
  aoai13aa1n02x5               g186(.a(new_n247), .b(new_n233), .c(new_n216), .d(new_n229), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n270), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n273), .b(new_n283), .c(new_n282), .d(new_n256), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n275), .b(new_n284), .c(new_n179), .d(new_n271), .o1(new_n285));
  tech160nm_fiaoi012aa1n02p5x5 g190(.a(new_n280), .b(new_n285), .c(new_n278), .o1(new_n286));
  norp02aa1n03x5               g191(.a(new_n286), .b(new_n281), .o1(\s[28] ));
  norb02aa1n02x5               g192(.a(new_n275), .b(new_n280), .out0(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n284), .c(new_n179), .d(new_n271), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[28] ), .b(\a[29] ), .out0(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n291), .b(new_n289), .c(new_n290), .o1(new_n292));
  aobi12aa1n02x7               g197(.a(new_n288), .b(new_n274), .c(new_n272), .out0(new_n293));
  nano22aa1n03x5               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  norp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g201(.a(new_n275), .b(new_n291), .c(new_n280), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n284), .c(new_n179), .d(new_n271), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  tech160nm_fiaoi012aa1n02p5x5 g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  aobi12aa1n02x7               g206(.a(new_n297), .b(new_n274), .c(new_n272), .out0(new_n302));
  nano22aa1n02x4               g207(.a(new_n302), .b(new_n299), .c(new_n300), .out0(new_n303));
  norp02aa1n03x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  norb02aa1n02x5               g210(.a(new_n297), .b(new_n300), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n284), .c(new_n179), .d(new_n271), .o1(new_n307));
  oao003aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n305), .b(new_n307), .c(new_n308), .o1(new_n309));
  aobi12aa1n02x7               g214(.a(new_n306), .b(new_n274), .c(new_n272), .out0(new_n310));
  nano22aa1n03x5               g215(.a(new_n310), .b(new_n305), .c(new_n308), .out0(new_n311));
  norp02aa1n03x5               g216(.a(new_n309), .b(new_n311), .o1(\s[31] ));
  oai012aa1n02x5               g217(.a(new_n99), .b(new_n100), .c(new_n98), .o1(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g219(.a(\a[3] ), .b(\b[2] ), .c(new_n313), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g221(.a(new_n104), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g222(.a(new_n109), .b(new_n104), .c(new_n108), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g224(.a(new_n107), .b(new_n106), .c(new_n318), .out0(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g226(.a(\a[7] ), .b(\b[6] ), .c(new_n320), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g228(.a(new_n120), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


