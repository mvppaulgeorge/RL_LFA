// Benchmark "adder" written by ABC on Wed Jul 17 17:22:45 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n329, new_n332, new_n333,
    new_n334, new_n336, new_n338, new_n339, new_n340, new_n341;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  aoi112aa1n03x5               g002(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n98));
  oab012aa1n02x4               g003(.a(new_n98), .b(\a[4] ), .c(\b[3] ), .out0(new_n99));
  inv000aa1d42x5               g004(.a(\a[2] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[1] ), .o1(new_n101));
  nand02aa1n08x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  oao003aa1n02x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .carry(new_n103));
  xorc02aa1n02x5               g008(.a(\a[4] ), .b(\b[3] ), .out0(new_n104));
  xorc02aa1n02x5               g009(.a(\a[3] ), .b(\b[2] ), .out0(new_n105));
  nanp03aa1n02x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n106));
  oai022aa1d24x5               g011(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n107));
  aoi022aa1n02x5               g012(.a(\b[6] ), .b(\a[7] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n108));
  nand02aa1d12x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanb02aa1n03x5               g015(.a(new_n110), .b(new_n109), .out0(new_n111));
  oai022aa1d24x5               g016(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n112));
  aoi012aa1d24x5               g017(.a(new_n112), .b(\a[6] ), .c(\b[5] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n108), .b(new_n113), .c(new_n111), .d(new_n107), .out0(new_n114));
  nand02aa1n03x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nand22aa1n03x5               g020(.a(new_n109), .b(new_n115), .o1(new_n116));
  norb02aa1n06x4               g021(.a(new_n107), .b(new_n116), .out0(new_n117));
  aoi022aa1d24x5               g022(.a(new_n117), .b(new_n113), .c(new_n109), .d(new_n112), .o1(new_n118));
  aoai13aa1n12x5               g023(.a(new_n118), .b(new_n114), .c(new_n106), .d(new_n99), .o1(new_n119));
  tech160nm_fixnrc02aa1n05x5   g024(.a(\b[8] ), .b(\a[9] ), .out0(new_n120));
  inv000aa1d42x5               g025(.a(new_n120), .o1(new_n121));
  xorc02aa1n02x5               g026(.a(\a[10] ), .b(\b[9] ), .out0(new_n122));
  aoai13aa1n06x5               g027(.a(new_n122), .b(new_n97), .c(new_n119), .d(new_n121), .o1(new_n123));
  aoi112aa1n02x5               g028(.a(new_n122), .b(new_n97), .c(new_n119), .d(new_n121), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n123), .b(new_n124), .out0(\s[10] ));
  inv000aa1d42x5               g030(.a(\a[10] ), .o1(new_n126));
  inv000aa1d42x5               g031(.a(\b[9] ), .o1(new_n127));
  oaoi03aa1n02x5               g032(.a(new_n126), .b(new_n127), .c(new_n97), .o1(new_n128));
  xnrc02aa1n12x5               g033(.a(\b[10] ), .b(\a[11] ), .out0(new_n129));
  xobna2aa1n03x5               g034(.a(new_n129), .b(new_n123), .c(new_n128), .out0(\s[11] ));
  oao003aa1n02x5               g035(.a(new_n126), .b(new_n127), .c(new_n97), .carry(new_n131));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norp02aa1n02x5               g037(.a(new_n131), .b(new_n132), .o1(new_n133));
  and002aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o(new_n134));
  nor042aa1n09x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nand22aa1n12x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanb02aa1d24x5               g041(.a(new_n135), .b(new_n136), .out0(new_n137));
  nanb02aa1n02x5               g042(.a(new_n134), .b(new_n137), .out0(new_n138));
  inv000aa1d42x5               g043(.a(new_n137), .o1(new_n139));
  aoai13aa1n03x5               g044(.a(new_n139), .b(new_n134), .c(new_n123), .d(new_n133), .o1(new_n140));
  aoai13aa1n02x5               g045(.a(new_n140), .b(new_n138), .c(new_n123), .d(new_n133), .o1(\s[12] ));
  oaoi03aa1n02x5               g046(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n142));
  xnrc02aa1n02x5               g047(.a(\b[3] ), .b(\a[4] ), .out0(new_n143));
  xnrc02aa1n02x5               g048(.a(\b[2] ), .b(\a[3] ), .out0(new_n144));
  oai013aa1n03x5               g049(.a(new_n99), .b(new_n142), .c(new_n144), .d(new_n143), .o1(new_n145));
  nano23aa1n03x7               g050(.a(new_n107), .b(new_n111), .c(new_n113), .d(new_n108), .out0(new_n146));
  inv000aa1d42x5               g051(.a(new_n118), .o1(new_n147));
  nor022aa1n04x5               g052(.a(new_n129), .b(new_n137), .o1(new_n148));
  nand23aa1n04x5               g053(.a(new_n148), .b(new_n121), .c(new_n122), .o1(new_n149));
  inv040aa1n02x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n06x5               g055(.a(new_n150), .b(new_n147), .c(new_n145), .d(new_n146), .o1(new_n151));
  oai012aa1n02x5               g056(.a(new_n136), .b(new_n135), .c(new_n132), .o1(new_n152));
  aobi12aa1n06x5               g057(.a(new_n152), .b(new_n148), .c(new_n131), .out0(new_n153));
  xorc02aa1n02x5               g058(.a(\a[13] ), .b(\b[12] ), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n151), .c(new_n153), .out0(\s[13] ));
  inv040aa1d30x5               g060(.a(\a[13] ), .o1(new_n156));
  nanb02aa1n02x5               g061(.a(\b[12] ), .b(new_n156), .out0(new_n157));
  oai013aa1n02x4               g062(.a(new_n152), .b(new_n128), .c(new_n129), .d(new_n137), .o1(new_n158));
  aoai13aa1n02x5               g063(.a(new_n154), .b(new_n158), .c(new_n119), .d(new_n150), .o1(new_n159));
  xorc02aa1n02x5               g064(.a(\a[14] ), .b(\b[13] ), .out0(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n160), .b(new_n159), .c(new_n157), .out0(\s[14] ));
  inv040aa1d32x5               g066(.a(\a[14] ), .o1(new_n162));
  xroi22aa1d06x4               g067(.a(new_n156), .b(\b[12] ), .c(new_n162), .d(\b[13] ), .out0(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  inv000aa1d42x5               g069(.a(\b[13] ), .o1(new_n165));
  norp02aa1n02x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  oaoi03aa1n02x5               g071(.a(new_n162), .b(new_n165), .c(new_n166), .o1(new_n167));
  aoai13aa1n06x5               g072(.a(new_n167), .b(new_n164), .c(new_n151), .d(new_n153), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  xnrc02aa1n12x5               g075(.a(\b[14] ), .b(\a[15] ), .out0(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  tech160nm_fixnrc02aa1n02p5x5 g077(.a(\b[15] ), .b(\a[16] ), .out0(new_n173));
  aoai13aa1n03x5               g078(.a(new_n173), .b(new_n170), .c(new_n168), .d(new_n172), .o1(new_n174));
  norp02aa1n02x5               g079(.a(new_n173), .b(new_n170), .o1(new_n175));
  aob012aa1n02x5               g080(.a(new_n175), .b(new_n168), .c(new_n172), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n174), .b(new_n176), .o1(\s[16] ));
  nor002aa1n02x5               g082(.a(new_n173), .b(new_n171), .o1(new_n178));
  nano22aa1d15x5               g083(.a(new_n149), .b(new_n163), .c(new_n178), .out0(new_n179));
  aoai13aa1n06x5               g084(.a(new_n179), .b(new_n147), .c(new_n145), .d(new_n146), .o1(new_n180));
  nand02aa1d04x5               g085(.a(new_n163), .b(new_n178), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n182));
  oab012aa1n02x4               g087(.a(new_n182), .b(\a[16] ), .c(\b[15] ), .out0(new_n183));
  oai013aa1n02x4               g088(.a(new_n183), .b(new_n167), .c(new_n171), .d(new_n173), .o1(new_n184));
  aoib12aa1n03x5               g089(.a(new_n184), .b(new_n158), .c(new_n181), .out0(new_n185));
  nand42aa1n06x5               g090(.a(new_n180), .b(new_n185), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  xnrc02aa1n02x5               g092(.a(\b[17] ), .b(\a[18] ), .out0(new_n188));
  nor042aa1n03x5               g093(.a(\b[16] ), .b(\a[17] ), .o1(new_n189));
  nona23aa1n02x4               g094(.a(new_n180), .b(new_n185), .c(new_n184), .d(new_n189), .out0(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  oaib12aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(\a[17] ), .out0(new_n192));
  inv000aa1d42x5               g097(.a(\a[17] ), .o1(new_n193));
  oai022aa1n02x5               g098(.a(new_n193), .b(new_n191), .c(\b[17] ), .d(\a[18] ), .o1(new_n194));
  aoi012aa1n02x5               g099(.a(new_n194), .b(\a[18] ), .c(\b[17] ), .o1(new_n195));
  aoi022aa1n02x5               g100(.a(new_n192), .b(new_n188), .c(new_n190), .d(new_n195), .o1(\s[18] ));
  oabi12aa1n12x5               g101(.a(new_n184), .b(new_n153), .c(new_n181), .out0(new_n197));
  inv000aa1d42x5               g102(.a(\a[18] ), .o1(new_n198));
  xroi22aa1d06x4               g103(.a(new_n193), .b(\b[16] ), .c(new_n198), .d(\b[17] ), .out0(new_n199));
  aoai13aa1n06x5               g104(.a(new_n199), .b(new_n197), .c(new_n119), .d(new_n179), .o1(new_n200));
  aoi112aa1n03x5               g105(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n201));
  aoib12aa1n06x5               g106(.a(new_n201), .b(new_n198), .c(\b[17] ), .out0(new_n202));
  nand22aa1n03x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nor002aa1d32x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nanb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n200), .c(new_n202), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g113(.a(new_n204), .o1(new_n209));
  aob012aa1n02x5               g114(.a(new_n189), .b(\b[17] ), .c(\a[18] ), .out0(new_n210));
  oaib12aa1n02x5               g115(.a(new_n210), .b(\b[17] ), .c(new_n198), .out0(new_n211));
  aoai13aa1n03x5               g116(.a(new_n206), .b(new_n211), .c(new_n186), .d(new_n199), .o1(new_n212));
  norp02aa1n06x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nand02aa1n03x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  norb03aa1n02x5               g120(.a(new_n214), .b(new_n204), .c(new_n213), .out0(new_n216));
  aoai13aa1n04x5               g121(.a(new_n216), .b(new_n205), .c(new_n200), .d(new_n202), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n217), .b(new_n215), .c(new_n212), .d(new_n209), .o1(\s[20] ));
  nano23aa1n06x5               g123(.a(new_n213), .b(new_n204), .c(new_n214), .d(new_n203), .out0(new_n219));
  nand02aa1n06x5               g124(.a(new_n199), .b(new_n219), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  aoai13aa1n06x5               g126(.a(new_n221), .b(new_n197), .c(new_n119), .d(new_n179), .o1(new_n222));
  nona23aa1n09x5               g127(.a(new_n203), .b(new_n214), .c(new_n213), .d(new_n204), .out0(new_n223));
  tech160nm_fioaoi03aa1n03p5x5 g128(.a(\a[20] ), .b(\b[19] ), .c(new_n209), .o1(new_n224));
  oabi12aa1n18x5               g129(.a(new_n224), .b(new_n223), .c(new_n202), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  xnrc02aa1n12x5               g131(.a(\b[20] ), .b(\a[21] ), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  xnbna2aa1n03x5               g133(.a(new_n228), .b(new_n222), .c(new_n226), .out0(\s[21] ));
  norp02aa1n02x5               g134(.a(\b[20] ), .b(\a[21] ), .o1(new_n230));
  inv000aa1n03x5               g135(.a(new_n230), .o1(new_n231));
  aoai13aa1n03x5               g136(.a(new_n228), .b(new_n225), .c(new_n186), .d(new_n221), .o1(new_n232));
  xnrc02aa1n12x5               g137(.a(\b[21] ), .b(\a[22] ), .out0(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  oai022aa1n02x5               g139(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(\a[22] ), .c(\b[21] ), .o1(new_n236));
  aoai13aa1n02x7               g141(.a(new_n236), .b(new_n227), .c(new_n222), .d(new_n226), .o1(new_n237));
  aoai13aa1n03x5               g142(.a(new_n237), .b(new_n234), .c(new_n232), .d(new_n231), .o1(\s[22] ));
  nor002aa1n06x5               g143(.a(new_n233), .b(new_n227), .o1(new_n239));
  and003aa1n02x5               g144(.a(new_n199), .b(new_n239), .c(new_n219), .o(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n197), .c(new_n119), .d(new_n179), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n239), .b(new_n224), .c(new_n219), .d(new_n211), .o1(new_n242));
  oaoi03aa1n09x5               g147(.a(\a[22] ), .b(\b[21] ), .c(new_n231), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  nanp02aa1n06x5               g149(.a(new_n242), .b(new_n244), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  nor022aa1n16x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  nand42aa1n03x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  nanb02aa1n06x5               g153(.a(new_n247), .b(new_n248), .out0(new_n249));
  xobna2aa1n03x5               g154(.a(new_n249), .b(new_n241), .c(new_n246), .out0(\s[23] ));
  xnrc02aa1n02x5               g155(.a(\b[23] ), .b(\a[24] ), .out0(new_n251));
  nona22aa1n02x4               g156(.a(new_n241), .b(new_n245), .c(new_n247), .out0(new_n252));
  nanp02aa1n02x5               g157(.a(new_n252), .b(new_n248), .o1(new_n253));
  oai012aa1n02x5               g158(.a(new_n248), .b(\b[23] ), .c(\a[24] ), .o1(new_n254));
  aoi012aa1n02x5               g159(.a(new_n254), .b(\a[24] ), .c(\b[23] ), .o1(new_n255));
  aoi022aa1n02x5               g160(.a(new_n253), .b(new_n251), .c(new_n252), .d(new_n255), .o1(\s[24] ));
  nor042aa1n03x5               g161(.a(new_n251), .b(new_n249), .o1(new_n257));
  nano22aa1n02x5               g162(.a(new_n220), .b(new_n239), .c(new_n257), .out0(new_n258));
  aoai13aa1n04x5               g163(.a(new_n258), .b(new_n197), .c(new_n119), .d(new_n179), .o1(new_n259));
  inv000aa1d42x5               g164(.a(\a[24] ), .o1(new_n260));
  inv000aa1d42x5               g165(.a(\b[23] ), .o1(new_n261));
  oao003aa1n02x5               g166(.a(new_n260), .b(new_n261), .c(new_n247), .carry(new_n262));
  aoi012aa1n06x5               g167(.a(new_n262), .b(new_n245), .c(new_n257), .o1(new_n263));
  xorc02aa1n12x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  xnbna2aa1n03x5               g169(.a(new_n264), .b(new_n259), .c(new_n263), .out0(\s[25] ));
  norp02aa1n02x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n257), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n262), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n269), .b(new_n268), .c(new_n242), .d(new_n244), .o1(new_n270));
  aoai13aa1n03x5               g175(.a(new_n264), .b(new_n270), .c(new_n186), .d(new_n258), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .out0(new_n272));
  inv000aa1d42x5               g177(.a(new_n264), .o1(new_n273));
  nanp02aa1n02x5               g178(.a(\b[25] ), .b(\a[26] ), .o1(new_n274));
  oai022aa1n02x5               g179(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n275));
  norb02aa1n02x5               g180(.a(new_n274), .b(new_n275), .out0(new_n276));
  aoai13aa1n02x7               g181(.a(new_n276), .b(new_n273), .c(new_n259), .d(new_n263), .o1(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n272), .c(new_n271), .d(new_n267), .o1(\s[26] ));
  and002aa1n06x5               g183(.a(new_n272), .b(new_n264), .o(new_n279));
  nano32aa1n03x7               g184(.a(new_n220), .b(new_n279), .c(new_n239), .d(new_n257), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n197), .c(new_n119), .d(new_n179), .o1(new_n281));
  aoi022aa1n02x5               g186(.a(new_n270), .b(new_n279), .c(new_n274), .d(new_n275), .o1(new_n282));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  nand42aa1n03x5               g188(.a(\b[26] ), .b(\a[27] ), .o1(new_n284));
  norb02aa1n06x4               g189(.a(new_n284), .b(new_n283), .out0(new_n285));
  xnbna2aa1n03x5               g190(.a(new_n285), .b(new_n282), .c(new_n281), .out0(\s[27] ));
  xnrc02aa1n02x5               g191(.a(\b[27] ), .b(\a[28] ), .out0(new_n287));
  inv000aa1d42x5               g192(.a(new_n279), .o1(new_n288));
  aoi012aa1n02x5               g193(.a(new_n283), .b(new_n275), .c(new_n274), .o1(new_n289));
  oai112aa1n03x5               g194(.a(new_n281), .b(new_n289), .c(new_n288), .d(new_n263), .o1(new_n290));
  nanp02aa1n03x5               g195(.a(new_n290), .b(new_n284), .o1(new_n291));
  oai012aa1n02x5               g196(.a(new_n284), .b(\b[27] ), .c(\a[28] ), .o1(new_n292));
  aoi012aa1n02x5               g197(.a(new_n292), .b(\a[28] ), .c(\b[27] ), .o1(new_n293));
  aoi022aa1n02x7               g198(.a(new_n291), .b(new_n287), .c(new_n290), .d(new_n293), .o1(\s[28] ));
  aoai13aa1n02x7               g199(.a(new_n257), .b(new_n243), .c(new_n225), .d(new_n239), .o1(new_n295));
  nanp02aa1n02x5               g200(.a(new_n275), .b(new_n274), .o1(new_n296));
  aoai13aa1n02x5               g201(.a(new_n296), .b(new_n288), .c(new_n295), .d(new_n269), .o1(new_n297));
  norb02aa1n06x5               g202(.a(new_n285), .b(new_n287), .out0(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n297), .c(new_n186), .d(new_n280), .o1(new_n299));
  inv020aa1n02x5               g204(.a(new_n298), .o1(new_n300));
  norp02aa1n02x5               g205(.a(\b[27] ), .b(\a[28] ), .o1(new_n301));
  aoi112aa1n02x5               g206(.a(\b[26] ), .b(\a[27] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n302));
  norp02aa1n02x5               g207(.a(new_n302), .b(new_n301), .o1(new_n303));
  aoai13aa1n02x7               g208(.a(new_n303), .b(new_n300), .c(new_n282), .d(new_n281), .o1(new_n304));
  tech160nm_fixorc02aa1n03p5x5 g209(.a(\a[29] ), .b(\b[28] ), .out0(new_n305));
  norp03aa1n02x5               g210(.a(new_n305), .b(new_n302), .c(new_n301), .o1(new_n306));
  aoi022aa1n03x5               g211(.a(new_n304), .b(new_n305), .c(new_n299), .d(new_n306), .o1(\s[29] ));
  xorb03aa1n02x5               g212(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g213(.a(new_n287), .b(new_n305), .c(new_n285), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n297), .c(new_n186), .d(new_n280), .o1(new_n310));
  inv000aa1n02x5               g215(.a(new_n309), .o1(new_n311));
  oaoi03aa1n02x5               g216(.a(\a[29] ), .b(\b[28] ), .c(new_n303), .o1(new_n312));
  inv000aa1n03x5               g217(.a(new_n312), .o1(new_n313));
  aoai13aa1n02x7               g218(.a(new_n313), .b(new_n311), .c(new_n282), .d(new_n281), .o1(new_n314));
  xorc02aa1n02x5               g219(.a(\a[30] ), .b(\b[29] ), .out0(new_n315));
  norp02aa1n02x5               g220(.a(\b[28] ), .b(\a[29] ), .o1(new_n316));
  aoi012aa1n02x5               g221(.a(new_n303), .b(\a[29] ), .c(\b[28] ), .o1(new_n317));
  norp03aa1n02x5               g222(.a(new_n317), .b(new_n315), .c(new_n316), .o1(new_n318));
  aoi022aa1n03x5               g223(.a(new_n314), .b(new_n315), .c(new_n310), .d(new_n318), .o1(\s[30] ));
  nano22aa1n06x5               g224(.a(new_n300), .b(new_n305), .c(new_n315), .out0(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n297), .c(new_n186), .d(new_n280), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n320), .o1(new_n322));
  oao003aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .c(new_n313), .carry(new_n323));
  aoai13aa1n02x7               g228(.a(new_n323), .b(new_n322), .c(new_n282), .d(new_n281), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[31] ), .b(\b[30] ), .out0(new_n325));
  norb02aa1n02x5               g230(.a(new_n323), .b(new_n325), .out0(new_n326));
  aoi022aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n321), .d(new_n326), .o1(\s[31] ));
  xnrc02aa1n02x5               g232(.a(new_n142), .b(new_n105), .out0(\s[3] ));
  oaoi03aa1n02x5               g233(.a(\a[3] ), .b(\b[2] ), .c(new_n142), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g235(.a(new_n145), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g236(.a(\a[5] ), .o1(new_n332));
  inv000aa1d42x5               g237(.a(\b[4] ), .o1(new_n333));
  tech160nm_fioaoi03aa1n03p5x5 g238(.a(new_n332), .b(new_n333), .c(new_n145), .o1(new_n334));
  xnrb03aa1n02x5               g239(.a(new_n334), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fioaoi03aa1n03p5x5 g240(.a(\a[6] ), .b(\b[5] ), .c(new_n334), .o1(new_n336));
  xorb03aa1n02x5               g241(.a(new_n336), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  xorc02aa1n02x5               g242(.a(\a[7] ), .b(\b[6] ), .out0(new_n338));
  nanb02aa1n03x5               g243(.a(new_n336), .b(new_n338), .out0(new_n339));
  oaib12aa1n02x5               g244(.a(new_n115), .b(new_n336), .c(new_n338), .out0(new_n340));
  nano22aa1n02x4               g245(.a(new_n110), .b(new_n115), .c(new_n109), .out0(new_n341));
  aoi022aa1n02x5               g246(.a(new_n340), .b(new_n111), .c(new_n339), .d(new_n341), .o1(\s[8] ));
  xorb03aa1n02x5               g247(.a(new_n119), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


