// Benchmark "adder" written by ABC on Thu Jul 18 11:05:12 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n152, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n160, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n180, new_n181,
    new_n182, new_n183, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n302, new_n305,
    new_n307, new_n308, new_n310, new_n311, new_n313;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nor042aa1n04x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nand42aa1n08x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norb02aa1n06x5               g004(.a(new_n99), .b(new_n98), .out0(new_n100));
  xorc02aa1n12x5               g005(.a(\a[7] ), .b(\b[6] ), .out0(new_n101));
  nand42aa1n02x5               g006(.a(\b[5] ), .b(\a[6] ), .o1(new_n102));
  nor022aa1n16x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  nor022aa1n04x5               g008(.a(\b[4] ), .b(\a[5] ), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nona23aa1n09x5               g010(.a(new_n102), .b(new_n105), .c(new_n104), .d(new_n103), .out0(new_n106));
  nano22aa1n06x5               g011(.a(new_n106), .b(new_n101), .c(new_n100), .out0(new_n107));
  tech160nm_finor002aa1n03p5x5 g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nand02aa1d04x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  nand22aa1n03x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  tech160nm_fiaoi012aa1n05x5   g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  nor042aa1n06x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nor022aa1n16x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nand42aa1n03x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nona23aa1n09x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  oai012aa1n02x7               g021(.a(new_n113), .b(new_n114), .c(new_n112), .o1(new_n117));
  oai012aa1n12x5               g022(.a(new_n117), .b(new_n116), .c(new_n111), .o1(new_n118));
  aoi112aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n119));
  aoi112aa1n09x5               g024(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n120));
  oai112aa1n04x5               g025(.a(new_n101), .b(new_n100), .c(new_n103), .d(new_n120), .o1(new_n121));
  nona22aa1n06x5               g026(.a(new_n121), .b(new_n119), .c(new_n98), .out0(new_n122));
  tech160nm_fixorc02aa1n03p5x5 g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n118), .d(new_n107), .o1(new_n124));
  tech160nm_fixorc02aa1n03p5x5 g029(.a(\a[10] ), .b(\b[9] ), .out0(new_n125));
  xnbna2aa1n03x5               g030(.a(new_n125), .b(new_n124), .c(new_n97), .out0(\s[10] ));
  nor002aa1n03x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  and002aa1n02x5               g032(.a(new_n125), .b(new_n123), .o(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n122), .c(new_n118), .d(new_n107), .o1(new_n129));
  aoi112aa1n09x5               g034(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n130));
  nona22aa1n02x4               g035(.a(new_n129), .b(new_n130), .c(new_n127), .out0(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n09x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand02aa1d28x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  aoi012aa1n02x5               g039(.a(new_n133), .b(new_n131), .c(new_n134), .o1(new_n135));
  nor042aa1n12x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nand22aa1n12x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n06x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  xnrc02aa1n02x5               g043(.a(new_n135), .b(new_n138), .out0(\s[12] ));
  nano23aa1n03x7               g044(.a(new_n133), .b(new_n136), .c(new_n137), .d(new_n134), .out0(new_n140));
  and003aa1n02x5               g045(.a(new_n140), .b(new_n125), .c(new_n123), .o(new_n141));
  aoai13aa1n06x5               g046(.a(new_n141), .b(new_n122), .c(new_n118), .d(new_n107), .o1(new_n142));
  aoi112aa1n09x5               g047(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n143));
  norb02aa1n15x5               g048(.a(new_n134), .b(new_n133), .out0(new_n144));
  oai112aa1n06x5               g049(.a(new_n138), .b(new_n144), .c(new_n130), .d(new_n127), .o1(new_n145));
  nona22aa1d18x5               g050(.a(new_n145), .b(new_n143), .c(new_n136), .out0(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(new_n142), .b(new_n147), .o1(new_n148));
  xorb03aa1n02x5               g053(.a(new_n148), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n04x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nand42aa1n08x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  aoi012aa1n02x5               g056(.a(new_n150), .b(new_n148), .c(new_n151), .o1(new_n152));
  xnrb03aa1n03x5               g057(.a(new_n152), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n03x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nand02aa1n10x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nona23aa1n03x5               g060(.a(new_n155), .b(new_n151), .c(new_n150), .d(new_n154), .out0(new_n156));
  aoi012aa1n02x7               g061(.a(new_n154), .b(new_n150), .c(new_n155), .o1(new_n157));
  aoai13aa1n03x5               g062(.a(new_n157), .b(new_n156), .c(new_n142), .d(new_n147), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n12x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nand02aa1n06x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  nor022aa1n06x5               g066(.a(\b[15] ), .b(\a[16] ), .o1(new_n162));
  nand02aa1n04x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  aoi112aa1n02x5               g069(.a(new_n164), .b(new_n160), .c(new_n158), .d(new_n161), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n164), .b(new_n160), .c(new_n158), .d(new_n161), .o1(new_n166));
  norb02aa1n03x4               g071(.a(new_n166), .b(new_n165), .out0(\s[16] ));
  nano23aa1n03x5               g072(.a(new_n150), .b(new_n154), .c(new_n155), .d(new_n151), .out0(new_n168));
  nano23aa1n03x5               g073(.a(new_n160), .b(new_n162), .c(new_n163), .d(new_n161), .out0(new_n169));
  nand02aa1n02x5               g074(.a(new_n169), .b(new_n168), .o1(new_n170));
  nano32aa1n03x7               g075(.a(new_n170), .b(new_n140), .c(new_n125), .d(new_n123), .out0(new_n171));
  aoai13aa1n12x5               g076(.a(new_n171), .b(new_n122), .c(new_n118), .d(new_n107), .o1(new_n172));
  nona23aa1n03x5               g077(.a(new_n163), .b(new_n161), .c(new_n160), .d(new_n162), .out0(new_n173));
  nor002aa1n03x5               g078(.a(new_n173), .b(new_n156), .o1(new_n174));
  aoi112aa1n02x5               g079(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n175));
  oai022aa1n02x5               g080(.a(new_n173), .b(new_n157), .c(\b[15] ), .d(\a[16] ), .o1(new_n176));
  aoi112aa1n09x5               g081(.a(new_n176), .b(new_n175), .c(new_n146), .d(new_n174), .o1(new_n177));
  nand02aa1d10x5               g082(.a(new_n177), .b(new_n172), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g084(.a(\a[18] ), .o1(new_n180));
  inv040aa1d30x5               g085(.a(\a[17] ), .o1(new_n181));
  inv000aa1d42x5               g086(.a(\b[16] ), .o1(new_n182));
  oaoi03aa1n03x5               g087(.a(new_n181), .b(new_n182), .c(new_n178), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[17] ), .c(new_n180), .out0(\s[18] ));
  xroi22aa1d06x4               g089(.a(new_n181), .b(\b[16] ), .c(new_n180), .d(\b[17] ), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  nona22aa1n02x4               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(new_n187));
  oaib12aa1n09x5               g092(.a(new_n187), .b(\b[17] ), .c(new_n180), .out0(new_n188));
  nor002aa1d32x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  nand42aa1n02x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n188), .c(new_n178), .d(new_n185), .o1(new_n192));
  aoi112aa1n02x5               g097(.a(new_n191), .b(new_n188), .c(new_n178), .d(new_n185), .o1(new_n193));
  norb02aa1n03x4               g098(.a(new_n192), .b(new_n193), .out0(\s[19] ));
  xnrc02aa1n02x5               g099(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  nand42aa1n02x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  nona22aa1n03x5               g103(.a(new_n192), .b(new_n198), .c(new_n189), .out0(new_n199));
  inv000aa1n06x5               g104(.a(new_n189), .o1(new_n200));
  aobi12aa1n06x5               g105(.a(new_n198), .b(new_n192), .c(new_n200), .out0(new_n201));
  norb02aa1n03x4               g106(.a(new_n199), .b(new_n201), .out0(\s[20] ));
  nano23aa1n09x5               g107(.a(new_n189), .b(new_n196), .c(new_n197), .d(new_n190), .out0(new_n203));
  nand22aa1n04x5               g108(.a(new_n185), .b(new_n203), .o1(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  norp02aa1n02x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  aoi013aa1n02x4               g111(.a(new_n206), .b(new_n186), .c(new_n181), .d(new_n182), .o1(new_n207));
  nona23aa1n02x4               g112(.a(new_n197), .b(new_n190), .c(new_n189), .d(new_n196), .out0(new_n208));
  oaoi03aa1n12x5               g113(.a(\a[20] ), .b(\b[19] ), .c(new_n200), .o1(new_n209));
  inv040aa1n03x5               g114(.a(new_n209), .o1(new_n210));
  oai012aa1n06x5               g115(.a(new_n210), .b(new_n208), .c(new_n207), .o1(new_n211));
  xorc02aa1n02x5               g116(.a(\a[21] ), .b(\b[20] ), .out0(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n211), .c(new_n178), .d(new_n205), .o1(new_n213));
  aoi112aa1n02x5               g118(.a(new_n212), .b(new_n211), .c(new_n178), .d(new_n205), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n213), .b(new_n214), .out0(\s[21] ));
  norp02aa1n04x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  inv040aa1n03x5               g121(.a(new_n216), .o1(new_n217));
  tech160nm_fixnrc02aa1n05x5   g122(.a(\b[21] ), .b(\a[22] ), .out0(new_n218));
  nanp03aa1n03x5               g123(.a(new_n213), .b(new_n217), .c(new_n218), .o1(new_n219));
  tech160nm_fiaoi012aa1n02p5x5 g124(.a(new_n218), .b(new_n213), .c(new_n217), .o1(new_n220));
  norb02aa1n03x4               g125(.a(new_n219), .b(new_n220), .out0(\s[22] ));
  nanp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  nano22aa1n03x7               g127(.a(new_n218), .b(new_n217), .c(new_n222), .out0(new_n223));
  and003aa1n02x5               g128(.a(new_n185), .b(new_n223), .c(new_n203), .o(new_n224));
  aoai13aa1n06x5               g129(.a(new_n223), .b(new_n209), .c(new_n203), .d(new_n188), .o1(new_n225));
  oaoi03aa1n02x5               g130(.a(\a[22] ), .b(\b[21] ), .c(new_n217), .o1(new_n226));
  inv000aa1n02x5               g131(.a(new_n226), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(new_n225), .b(new_n227), .o1(new_n228));
  xorc02aa1n02x5               g133(.a(\a[23] ), .b(\b[22] ), .out0(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n228), .c(new_n178), .d(new_n224), .o1(new_n230));
  aoi112aa1n02x5               g135(.a(new_n229), .b(new_n228), .c(new_n178), .d(new_n224), .o1(new_n231));
  norb02aa1n03x4               g136(.a(new_n230), .b(new_n231), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  nona22aa1n03x5               g139(.a(new_n230), .b(new_n234), .c(new_n233), .out0(new_n235));
  inv000aa1n02x5               g140(.a(new_n233), .o1(new_n236));
  aobi12aa1n06x5               g141(.a(new_n234), .b(new_n230), .c(new_n236), .out0(new_n237));
  norb02aa1n03x4               g142(.a(new_n235), .b(new_n237), .out0(\s[24] ));
  inv000aa1d42x5               g143(.a(\a[23] ), .o1(new_n239));
  inv020aa1n04x5               g144(.a(\a[24] ), .o1(new_n240));
  xroi22aa1d06x4               g145(.a(new_n239), .b(\b[22] ), .c(new_n240), .d(\b[23] ), .out0(new_n241));
  nanb03aa1n03x5               g146(.a(new_n204), .b(new_n241), .c(new_n223), .out0(new_n242));
  inv000aa1n02x5               g147(.a(new_n241), .o1(new_n243));
  oao003aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .c(new_n236), .carry(new_n244));
  aoai13aa1n12x5               g149(.a(new_n244), .b(new_n243), .c(new_n225), .d(new_n227), .o1(new_n245));
  inv040aa1n03x5               g150(.a(new_n245), .o1(new_n246));
  aoai13aa1n04x5               g151(.a(new_n246), .b(new_n242), .c(new_n172), .d(new_n177), .o1(new_n247));
  xorb03aa1n02x5               g152(.a(new_n247), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g153(.a(\b[24] ), .b(\a[25] ), .o1(new_n249));
  xorc02aa1n12x5               g154(.a(\a[25] ), .b(\b[24] ), .out0(new_n250));
  nor042aa1n02x5               g155(.a(\b[25] ), .b(\a[26] ), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(\b[25] ), .b(\a[26] ), .o1(new_n252));
  norb02aa1n06x5               g157(.a(new_n252), .b(new_n251), .out0(new_n253));
  aoi112aa1n02x5               g158(.a(new_n249), .b(new_n253), .c(new_n247), .d(new_n250), .o1(new_n254));
  aoai13aa1n03x5               g159(.a(new_n253), .b(new_n249), .c(new_n247), .d(new_n250), .o1(new_n255));
  norb02aa1n02x7               g160(.a(new_n255), .b(new_n254), .out0(\s[26] ));
  nand42aa1n02x5               g161(.a(new_n250), .b(new_n253), .o1(new_n257));
  inv000aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  nano32aa1n03x7               g163(.a(new_n204), .b(new_n258), .c(new_n223), .d(new_n241), .out0(new_n259));
  nand22aa1n12x5               g164(.a(new_n178), .b(new_n259), .o1(new_n260));
  oai012aa1n02x5               g165(.a(new_n252), .b(new_n251), .c(new_n249), .o1(new_n261));
  aobi12aa1n12x5               g166(.a(new_n261), .b(new_n245), .c(new_n258), .out0(new_n262));
  xorc02aa1n12x5               g167(.a(\a[27] ), .b(\b[26] ), .out0(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n263), .b(new_n262), .c(new_n260), .out0(\s[27] ));
  norp02aa1n02x5               g169(.a(\b[26] ), .b(\a[27] ), .o1(new_n265));
  inv040aa1n03x5               g170(.a(new_n265), .o1(new_n266));
  aobi12aa1n06x5               g171(.a(new_n263), .b(new_n262), .c(new_n260), .out0(new_n267));
  xnrc02aa1n02x5               g172(.a(\b[27] ), .b(\a[28] ), .out0(new_n268));
  nano22aa1n03x7               g173(.a(new_n267), .b(new_n266), .c(new_n268), .out0(new_n269));
  aobi12aa1n06x5               g174(.a(new_n259), .b(new_n177), .c(new_n172), .out0(new_n270));
  aoai13aa1n02x5               g175(.a(new_n241), .b(new_n226), .c(new_n211), .d(new_n223), .o1(new_n271));
  aoai13aa1n03x5               g176(.a(new_n261), .b(new_n257), .c(new_n271), .d(new_n244), .o1(new_n272));
  oaih12aa1n02x5               g177(.a(new_n263), .b(new_n272), .c(new_n270), .o1(new_n273));
  tech160nm_fiaoi012aa1n02p5x5 g178(.a(new_n268), .b(new_n273), .c(new_n266), .o1(new_n274));
  norp02aa1n03x5               g179(.a(new_n274), .b(new_n269), .o1(\s[28] ));
  norb02aa1n02x5               g180(.a(new_n263), .b(new_n268), .out0(new_n276));
  aobi12aa1n06x5               g181(.a(new_n276), .b(new_n262), .c(new_n260), .out0(new_n277));
  oao003aa1n02x5               g182(.a(\a[28] ), .b(\b[27] ), .c(new_n266), .carry(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[28] ), .b(\a[29] ), .out0(new_n279));
  nano22aa1n03x5               g184(.a(new_n277), .b(new_n278), .c(new_n279), .out0(new_n280));
  oaih12aa1n02x5               g185(.a(new_n276), .b(new_n272), .c(new_n270), .o1(new_n281));
  tech160nm_fiaoi012aa1n02p5x5 g186(.a(new_n279), .b(new_n281), .c(new_n278), .o1(new_n282));
  norp02aa1n03x5               g187(.a(new_n282), .b(new_n280), .o1(\s[29] ));
  xorb03aa1n02x5               g188(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g189(.a(new_n263), .b(new_n279), .c(new_n268), .out0(new_n285));
  aobi12aa1n06x5               g190(.a(new_n285), .b(new_n262), .c(new_n260), .out0(new_n286));
  oao003aa1n02x5               g191(.a(\a[29] ), .b(\b[28] ), .c(new_n278), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[29] ), .b(\a[30] ), .out0(new_n288));
  nano22aa1n03x5               g193(.a(new_n286), .b(new_n287), .c(new_n288), .out0(new_n289));
  oaih12aa1n02x5               g194(.a(new_n285), .b(new_n272), .c(new_n270), .o1(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n288), .b(new_n290), .c(new_n287), .o1(new_n291));
  norp02aa1n03x5               g196(.a(new_n291), .b(new_n289), .o1(\s[30] ));
  norb02aa1n02x5               g197(.a(new_n285), .b(new_n288), .out0(new_n293));
  aobi12aa1n06x5               g198(.a(new_n293), .b(new_n262), .c(new_n260), .out0(new_n294));
  oao003aa1n02x5               g199(.a(\a[30] ), .b(\b[29] ), .c(new_n287), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[30] ), .b(\a[31] ), .out0(new_n296));
  nano22aa1n03x5               g201(.a(new_n294), .b(new_n295), .c(new_n296), .out0(new_n297));
  oaih12aa1n02x5               g202(.a(new_n293), .b(new_n272), .c(new_n270), .o1(new_n298));
  tech160nm_fiaoi012aa1n02p5x5 g203(.a(new_n296), .b(new_n298), .c(new_n295), .o1(new_n299));
  norp02aa1n03x5               g204(.a(new_n299), .b(new_n297), .o1(\s[31] ));
  xnrb03aa1n02x5               g205(.a(new_n111), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g206(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n302));
  xorb03aa1n02x5               g207(.a(new_n302), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g208(.a(new_n118), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g209(.a(new_n104), .b(new_n118), .c(new_n105), .o1(new_n305));
  xnrb03aa1n02x5               g210(.a(new_n305), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi13aa1n02x5               g211(.a(new_n106), .b(new_n117), .c(new_n116), .d(new_n111), .o1(new_n307));
  orn003aa1n02x5               g212(.a(new_n307), .b(new_n103), .c(new_n120), .o(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  orn002aa1n02x5               g214(.a(\a[7] ), .b(\b[6] ), .o(new_n310));
  oai013aa1n02x4               g215(.a(new_n101), .b(new_n307), .c(new_n103), .d(new_n120), .o1(new_n311));
  xnbna2aa1n03x5               g216(.a(new_n100), .b(new_n311), .c(new_n310), .out0(\s[8] ));
  aoi112aa1n02x5               g217(.a(new_n122), .b(new_n123), .c(new_n118), .d(new_n107), .o1(new_n313));
  norb02aa1n02x5               g218(.a(new_n124), .b(new_n313), .out0(\s[9] ));
endmodule


