// Benchmark "adder" written by ABC on Thu Jul 18 00:39:02 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n314, new_n315, new_n317, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[2] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[1] ), .o1(new_n98));
  nand22aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  oaoi03aa1n12x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  nor002aa1n10x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand02aa1n06x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor022aa1n16x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand02aa1n03x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n09x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  tech160nm_fiaoi012aa1n03p5x5 g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  oai012aa1n12x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  norp02aa1n12x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nand22aa1n06x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n04x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1d18x5               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xorc02aa1n12x5               g017(.a(\a[6] ), .b(\b[5] ), .out0(new_n113));
  xorc02aa1n03x5               g018(.a(\a[5] ), .b(\b[4] ), .out0(new_n114));
  nano22aa1n12x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .out0(new_n115));
  norp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  aoi112aa1n03x5               g021(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n117));
  nor022aa1n04x5               g022(.a(new_n117), .b(new_n116), .o1(new_n118));
  oaih12aa1n02x5               g023(.a(new_n109), .b(new_n110), .c(new_n108), .o1(new_n119));
  tech160nm_fioai012aa1n05x5   g024(.a(new_n119), .b(new_n112), .c(new_n118), .o1(new_n120));
  tech160nm_fiaoi012aa1n05x5   g025(.a(new_n120), .b(new_n115), .c(new_n107), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .c(new_n121), .o1(new_n122));
  xorb03aa1n02x5               g027(.a(new_n122), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n06x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nand42aa1d28x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nor042aa1n12x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nanp02aa1n12x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nano23aa1d15x5               g032(.a(new_n124), .b(new_n126), .c(new_n127), .d(new_n125), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n120), .c(new_n115), .d(new_n107), .o1(new_n129));
  tech160nm_fiaoi012aa1n05x5   g034(.a(new_n124), .b(new_n126), .c(new_n125), .o1(new_n130));
  nor002aa1d32x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand42aa1d28x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n129), .c(new_n130), .out0(\s[11] ));
  inv040aa1n06x5               g039(.a(new_n131), .o1(new_n135));
  nand42aa1n02x5               g040(.a(new_n115), .b(new_n107), .o1(new_n136));
  inv020aa1n03x5               g041(.a(new_n120), .o1(new_n137));
  nand22aa1n04x5               g042(.a(new_n136), .b(new_n137), .o1(new_n138));
  inv000aa1n06x5               g043(.a(new_n130), .o1(new_n139));
  aoai13aa1n03x5               g044(.a(new_n133), .b(new_n139), .c(new_n138), .d(new_n128), .o1(new_n140));
  nor022aa1n16x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1d16x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n140), .c(new_n135), .out0(\s[12] ));
  inv000aa1d42x5               g049(.a(\a[13] ), .o1(new_n145));
  nano23aa1d15x5               g050(.a(new_n131), .b(new_n141), .c(new_n142), .d(new_n132), .out0(new_n146));
  nona23aa1n03x5               g051(.a(new_n142), .b(new_n132), .c(new_n131), .d(new_n141), .out0(new_n147));
  oaoi03aa1n09x5               g052(.a(\a[12] ), .b(\b[11] ), .c(new_n135), .o1(new_n148));
  oabi12aa1n03x5               g053(.a(new_n148), .b(new_n147), .c(new_n130), .out0(new_n149));
  aoi013aa1n03x5               g054(.a(new_n149), .b(new_n138), .c(new_n128), .d(new_n146), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(new_n145), .out0(\s[13] ));
  oaoi03aa1n03x5               g056(.a(\a[13] ), .b(\b[12] ), .c(new_n150), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1d18x5               g058(.a(\b[14] ), .b(\a[15] ), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  nand42aa1d28x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  nor042aa1n03x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand02aa1d16x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nor042aa1n04x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nand02aa1d08x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nano23aa1d15x5               g065(.a(new_n157), .b(new_n159), .c(new_n160), .d(new_n158), .out0(new_n161));
  nano32aa1n03x7               g066(.a(new_n121), .b(new_n161), .c(new_n128), .d(new_n146), .out0(new_n162));
  aoai13aa1n06x5               g067(.a(new_n161), .b(new_n148), .c(new_n146), .d(new_n139), .o1(new_n163));
  inv000aa1d42x5               g068(.a(\b[12] ), .o1(new_n164));
  aoai13aa1n04x5               g069(.a(new_n160), .b(new_n159), .c(new_n145), .d(new_n164), .o1(new_n165));
  nand02aa1d04x5               g070(.a(new_n163), .b(new_n165), .o1(new_n166));
  oai112aa1n04x5               g071(.a(new_n155), .b(new_n156), .c(new_n162), .d(new_n166), .o1(new_n167));
  aoi112aa1n02x5               g072(.a(new_n162), .b(new_n166), .c(new_n155), .d(new_n156), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(\s[15] ));
  nor042aa1n06x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand42aa1d28x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n167), .c(new_n155), .out0(\s[16] ));
  nano23aa1n09x5               g078(.a(new_n154), .b(new_n170), .c(new_n171), .d(new_n156), .out0(new_n174));
  inv000aa1n02x5               g079(.a(new_n174), .o1(new_n175));
  nano32aa1n03x7               g080(.a(new_n175), .b(new_n161), .c(new_n146), .d(new_n128), .out0(new_n176));
  aoai13aa1n12x5               g081(.a(new_n176), .b(new_n120), .c(new_n107), .d(new_n115), .o1(new_n177));
  inv000aa1n02x5               g082(.a(new_n165), .o1(new_n178));
  aoai13aa1n06x5               g083(.a(new_n174), .b(new_n178), .c(new_n149), .d(new_n161), .o1(new_n179));
  tech160nm_fioai012aa1n05x5   g084(.a(new_n171), .b(new_n170), .c(new_n154), .o1(new_n180));
  nand23aa1d12x5               g085(.a(new_n177), .b(new_n179), .c(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1d18x5               g087(.a(\b[16] ), .b(\a[17] ), .o1(new_n183));
  nand02aa1d24x5               g088(.a(\b[16] ), .b(\a[17] ), .o1(new_n184));
  tech160nm_fioai012aa1n05x5   g089(.a(new_n184), .b(new_n181), .c(new_n183), .o1(new_n185));
  xnrb03aa1n03x5               g090(.a(new_n185), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  aobi12aa1n12x5               g091(.a(new_n180), .b(new_n166), .c(new_n174), .out0(new_n187));
  nor042aa1d18x5               g092(.a(\b[17] ), .b(\a[18] ), .o1(new_n188));
  nand02aa1d28x5               g093(.a(\b[17] ), .b(\a[18] ), .o1(new_n189));
  nano23aa1d15x5               g094(.a(new_n183), .b(new_n188), .c(new_n189), .d(new_n184), .out0(new_n190));
  inv000aa1d42x5               g095(.a(new_n190), .o1(new_n191));
  aoi012aa1d18x5               g096(.a(new_n188), .b(new_n183), .c(new_n189), .o1(new_n192));
  aoai13aa1n04x5               g097(.a(new_n192), .b(new_n191), .c(new_n187), .d(new_n177), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g099(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanp02aa1n12x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  nor002aa1d32x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nand02aa1d06x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  aoi112aa1n03x4               g106(.a(new_n196), .b(new_n201), .c(new_n193), .d(new_n198), .o1(new_n202));
  inv040aa1n08x5               g107(.a(new_n196), .o1(new_n203));
  inv020aa1n02x5               g108(.a(new_n192), .o1(new_n204));
  aoai13aa1n03x5               g109(.a(new_n198), .b(new_n204), .c(new_n181), .d(new_n190), .o1(new_n205));
  aobi12aa1n06x5               g110(.a(new_n201), .b(new_n205), .c(new_n203), .out0(new_n206));
  nor002aa1n02x5               g111(.a(new_n206), .b(new_n202), .o1(\s[20] ));
  nano23aa1n09x5               g112(.a(new_n196), .b(new_n199), .c(new_n200), .d(new_n197), .out0(new_n208));
  nand22aa1n09x5               g113(.a(new_n208), .b(new_n190), .o1(new_n209));
  nona23aa1n09x5               g114(.a(new_n200), .b(new_n197), .c(new_n196), .d(new_n199), .out0(new_n210));
  oaoi03aa1n09x5               g115(.a(\a[20] ), .b(\b[19] ), .c(new_n203), .o1(new_n211));
  inv040aa1n03x5               g116(.a(new_n211), .o1(new_n212));
  oai012aa1d24x5               g117(.a(new_n212), .b(new_n210), .c(new_n192), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n04x5               g119(.a(new_n214), .b(new_n209), .c(new_n187), .d(new_n177), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xorc02aa1n02x5               g123(.a(\a[22] ), .b(\b[21] ), .out0(new_n219));
  aoi112aa1n03x4               g124(.a(new_n217), .b(new_n219), .c(new_n215), .d(new_n218), .o1(new_n220));
  inv000aa1n06x5               g125(.a(new_n217), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n209), .o1(new_n222));
  aoai13aa1n03x5               g127(.a(new_n218), .b(new_n213), .c(new_n181), .d(new_n222), .o1(new_n223));
  aobi12aa1n06x5               g128(.a(new_n219), .b(new_n223), .c(new_n221), .out0(new_n224));
  nor002aa1n02x5               g129(.a(new_n224), .b(new_n220), .o1(\s[22] ));
  inv040aa1d30x5               g130(.a(\a[21] ), .o1(new_n226));
  inv040aa1d32x5               g131(.a(\a[22] ), .o1(new_n227));
  xroi22aa1d06x4               g132(.a(new_n226), .b(\b[20] ), .c(new_n227), .d(\b[21] ), .out0(new_n228));
  oao003aa1n02x5               g133(.a(\a[22] ), .b(\b[21] ), .c(new_n221), .carry(new_n229));
  inv000aa1n02x5               g134(.a(new_n229), .o1(new_n230));
  aoi012aa1d18x5               g135(.a(new_n230), .b(new_n213), .c(new_n228), .o1(new_n231));
  nano22aa1n02x4               g136(.a(new_n209), .b(new_n218), .c(new_n219), .out0(new_n232));
  inv000aa1n02x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n04x5               g138(.a(new_n231), .b(new_n233), .c(new_n187), .d(new_n177), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  xorc02aa1n12x5               g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  xorc02aa1n12x5               g142(.a(\a[24] ), .b(\b[23] ), .out0(new_n238));
  aoi112aa1n03x4               g143(.a(new_n236), .b(new_n238), .c(new_n234), .d(new_n237), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n236), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n231), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n237), .b(new_n241), .c(new_n181), .d(new_n232), .o1(new_n242));
  aobi12aa1n06x5               g147(.a(new_n238), .b(new_n242), .c(new_n240), .out0(new_n243));
  nor042aa1n03x5               g148(.a(new_n243), .b(new_n239), .o1(\s[24] ));
  and002aa1n03x5               g149(.a(new_n238), .b(new_n237), .o(new_n245));
  inv040aa1n03x5               g150(.a(new_n245), .o1(new_n246));
  nano32aa1n02x4               g151(.a(new_n246), .b(new_n228), .c(new_n208), .d(new_n190), .out0(new_n247));
  inv000aa1n02x5               g152(.a(new_n247), .o1(new_n248));
  aoai13aa1n06x5               g153(.a(new_n228), .b(new_n211), .c(new_n208), .d(new_n204), .o1(new_n249));
  nanp02aa1n02x5               g154(.a(\b[23] ), .b(\a[24] ), .o1(new_n250));
  oai022aa1n02x5               g155(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(new_n251), .b(new_n250), .o1(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n246), .c(new_n249), .d(new_n229), .o1(new_n253));
  inv000aa1n02x5               g158(.a(new_n253), .o1(new_n254));
  aoai13aa1n04x5               g159(.a(new_n254), .b(new_n248), .c(new_n187), .d(new_n177), .o1(new_n255));
  xorb03aa1n02x5               g160(.a(new_n255), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  xorc02aa1n12x5               g162(.a(\a[25] ), .b(\b[24] ), .out0(new_n258));
  xorc02aa1n12x5               g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  aoi112aa1n03x4               g164(.a(new_n257), .b(new_n259), .c(new_n255), .d(new_n258), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n257), .o1(new_n261));
  aoai13aa1n03x5               g166(.a(new_n258), .b(new_n253), .c(new_n181), .d(new_n247), .o1(new_n262));
  aobi12aa1n06x5               g167(.a(new_n259), .b(new_n262), .c(new_n261), .out0(new_n263));
  nor002aa1n02x5               g168(.a(new_n263), .b(new_n260), .o1(\s[26] ));
  aoai13aa1n03x5               g169(.a(new_n180), .b(new_n175), .c(new_n163), .d(new_n165), .o1(new_n265));
  and002aa1n09x5               g170(.a(new_n259), .b(new_n258), .o(new_n266));
  nano23aa1n09x5               g171(.a(new_n209), .b(new_n246), .c(new_n266), .d(new_n228), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n265), .c(new_n138), .d(new_n176), .o1(new_n268));
  oao003aa1n02x5               g173(.a(\a[26] ), .b(\b[25] ), .c(new_n261), .carry(new_n269));
  aobi12aa1n06x5               g174(.a(new_n269), .b(new_n253), .c(new_n266), .out0(new_n270));
  nor042aa1n06x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  nand42aa1n03x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  norb02aa1n02x5               g177(.a(new_n272), .b(new_n271), .out0(new_n273));
  xnbna2aa1n03x5               g178(.a(new_n273), .b(new_n268), .c(new_n270), .out0(\s[27] ));
  xorc02aa1n02x5               g179(.a(\a[28] ), .b(\b[27] ), .out0(new_n275));
  aoai13aa1n04x5               g180(.a(new_n245), .b(new_n230), .c(new_n213), .d(new_n228), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n266), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n269), .b(new_n277), .c(new_n276), .d(new_n252), .o1(new_n278));
  aoi112aa1n03x4               g183(.a(new_n278), .b(new_n271), .c(new_n181), .d(new_n267), .o1(new_n279));
  nano22aa1n03x7               g184(.a(new_n279), .b(new_n272), .c(new_n275), .out0(new_n280));
  inv000aa1d42x5               g185(.a(new_n271), .o1(new_n281));
  nanp03aa1n03x5               g186(.a(new_n268), .b(new_n270), .c(new_n281), .o1(new_n282));
  aoi012aa1n03x5               g187(.a(new_n275), .b(new_n282), .c(new_n272), .o1(new_n283));
  nor042aa1n03x5               g188(.a(new_n283), .b(new_n280), .o1(\s[28] ));
  and002aa1n02x5               g189(.a(new_n275), .b(new_n273), .o(new_n285));
  aoai13aa1n03x5               g190(.a(new_n285), .b(new_n278), .c(new_n181), .d(new_n267), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n281), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  tech160nm_fiaoi012aa1n03p5x5 g193(.a(new_n288), .b(new_n286), .c(new_n287), .o1(new_n289));
  aobi12aa1n02x7               g194(.a(new_n285), .b(new_n268), .c(new_n270), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n287), .c(new_n288), .out0(new_n291));
  norp02aa1n03x5               g196(.a(new_n289), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g198(.a(new_n288), .b(new_n275), .c(new_n273), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n278), .c(new_n181), .d(new_n267), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  tech160nm_fiaoi012aa1n03p5x5 g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n02x7               g203(.a(new_n294), .b(new_n268), .c(new_n270), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n03x5               g205(.a(new_n298), .b(new_n300), .o1(\s[30] ));
  nano23aa1n02x4               g206(.a(new_n297), .b(new_n288), .c(new_n275), .d(new_n273), .out0(new_n302));
  aobi12aa1n02x7               g207(.a(new_n302), .b(new_n268), .c(new_n270), .out0(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nano22aa1n02x4               g210(.a(new_n303), .b(new_n304), .c(new_n305), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n302), .b(new_n278), .c(new_n181), .d(new_n267), .o1(new_n307));
  tech160nm_fiaoi012aa1n03p5x5 g212(.a(new_n305), .b(new_n307), .c(new_n304), .o1(new_n308));
  norp02aa1n03x5               g213(.a(new_n308), .b(new_n306), .o1(\s[31] ));
  xnrb03aa1n02x5               g214(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g215(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai112aa1n03x5               g218(.a(new_n106), .b(new_n114), .c(new_n105), .d(new_n100), .o1(new_n314));
  aob012aa1n03x5               g219(.a(new_n314), .b(\b[4] ), .c(\a[5] ), .out0(new_n315));
  xnrc02aa1n02x5               g220(.a(new_n315), .b(new_n113), .out0(\s[6] ));
  oao003aa1n03x5               g221(.a(\a[6] ), .b(\b[5] ), .c(new_n315), .carry(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n03x5               g223(.a(\a[7] ), .b(\b[6] ), .c(new_n317), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g225(.a(new_n121), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


