// Benchmark "adder" written by ABC on Wed Jul 10 16:49:56 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n147, new_n148, new_n149,
    new_n151, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n227, new_n228, new_n229, new_n230,
    new_n231, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n247, new_n248, new_n249, new_n250, new_n251, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n303,
    new_n306, new_n308, new_n309, new_n310, new_n311, new_n313;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  oai012aa1n02x5               g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n109), .b(new_n117), .o1(new_n118));
  nano23aa1n02x4               g023(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n119));
  orn002aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .o(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n122));
  aobi12aa1n02x5               g027(.a(new_n122), .b(new_n119), .c(new_n121), .out0(new_n123));
  xorc02aa1n02x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  160nm_ficinv00aa1n08x5       g029(.clk(new_n124), .clkout(new_n125));
  aoai13aa1n02x5               g030(.a(new_n98), .b(new_n125), .c(new_n118), .d(new_n123), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  xorc02aa1n02x5               g032(.a(\a[10] ), .b(\b[9] ), .out0(new_n128));
  norp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  oaoi03aa1n02x5               g036(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n132));
  aoai13aa1n02x5               g037(.a(new_n131), .b(new_n132), .c(new_n126), .d(new_n128), .o1(new_n133));
  aoi112aa1n02x5               g038(.a(new_n132), .b(new_n131), .c(new_n126), .d(new_n128), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n129), .clkout(new_n136));
  norp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n139), .b(new_n133), .c(new_n136), .out0(\s[12] ));
  nano23aa1n02x4               g045(.a(new_n129), .b(new_n137), .c(new_n138), .d(new_n130), .out0(new_n141));
  nanp03aa1n02x5               g046(.a(new_n141), .b(new_n124), .c(new_n128), .o1(new_n142));
  oaoi03aa1n02x5               g047(.a(\a[12] ), .b(\b[11] ), .c(new_n136), .o1(new_n143));
  aoi012aa1n02x5               g048(.a(new_n143), .b(new_n141), .c(new_n132), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n142), .c(new_n118), .d(new_n123), .o1(new_n145));
  xorb03aa1n02x5               g050(.a(new_n145), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g051(.a(\b[12] ), .b(\a[13] ), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  aoi012aa1n02x5               g053(.a(new_n147), .b(new_n145), .c(new_n148), .o1(new_n149));
  xnrb03aa1n02x5               g054(.a(new_n149), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g055(.a(\b[14] ), .b(\a[15] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[14] ), .b(\a[15] ), .o1(new_n152));
  nanb02aa1n02x5               g057(.a(new_n151), .b(new_n152), .out0(new_n153));
  160nm_ficinv00aa1n08x5       g058(.clk(new_n153), .clkout(new_n154));
  norp02aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nano23aa1n02x4               g061(.a(new_n147), .b(new_n155), .c(new_n156), .d(new_n148), .out0(new_n157));
  aoi012aa1n02x5               g062(.a(new_n155), .b(new_n147), .c(new_n156), .o1(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n158), .clkout(new_n159));
  aoai13aa1n02x5               g064(.a(new_n154), .b(new_n159), .c(new_n145), .d(new_n157), .o1(new_n160));
  aoi112aa1n02x5               g065(.a(new_n154), .b(new_n159), .c(new_n145), .d(new_n157), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n160), .b(new_n161), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g067(.clk(new_n151), .clkout(new_n163));
  norp02aa1n02x5               g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  nanb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(new_n166));
  xobna2aa1n03x5               g071(.a(new_n166), .b(new_n160), .c(new_n163), .out0(\s[16] ));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n123), .clkout(new_n168));
  nano23aa1n02x4               g073(.a(new_n151), .b(new_n164), .c(new_n165), .d(new_n152), .out0(new_n169));
  nano22aa1n02x4               g074(.a(new_n142), .b(new_n157), .c(new_n169), .out0(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n168), .c(new_n109), .d(new_n117), .o1(new_n171));
  norp02aa1n02x5               g076(.a(\b[9] ), .b(\a[10] ), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n173));
  oai112aa1n02x5               g078(.a(new_n139), .b(new_n131), .c(new_n173), .d(new_n172), .o1(new_n174));
  160nm_ficinv00aa1n08x5       g079(.clk(new_n143), .clkout(new_n175));
  160nm_ficinv00aa1n08x5       g080(.clk(new_n157), .clkout(new_n176));
  aoai13aa1n02x5               g081(.a(new_n158), .b(new_n176), .c(new_n174), .d(new_n175), .o1(new_n177));
  oai012aa1n02x5               g082(.a(new_n165), .b(new_n164), .c(new_n151), .o1(new_n178));
  aobi12aa1n02x5               g083(.a(new_n178), .b(new_n177), .c(new_n169), .out0(new_n179));
  xnrc02aa1n02x5               g084(.a(\b[16] ), .b(\a[17] ), .out0(new_n180));
  xobna2aa1n03x5               g085(.a(new_n180), .b(new_n179), .c(new_n171), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g086(.clk(\a[17] ), .clkout(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(\b[16] ), .clkout(new_n183));
  nanp02aa1n02x5               g088(.a(new_n183), .b(new_n182), .o1(new_n184));
  aoai13aa1n02x5               g089(.a(new_n184), .b(new_n180), .c(new_n179), .d(new_n171), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[18] ), .clkout(new_n187));
  xroi22aa1d04x5               g092(.a(new_n182), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(new_n188), .clkout(new_n189));
  norp02aa1n02x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nanp02aa1n02x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  aoi013aa1n02x4               g096(.a(new_n190), .b(new_n191), .c(new_n182), .d(new_n183), .o1(new_n192));
  aoai13aa1n02x5               g097(.a(new_n192), .b(new_n189), .c(new_n179), .d(new_n171), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g099(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  norp02aa1n02x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  aoi112aa1n02x5               g105(.a(new_n196), .b(new_n200), .c(new_n193), .d(new_n197), .o1(new_n201));
  aoai13aa1n02x5               g106(.a(new_n200), .b(new_n196), .c(new_n193), .d(new_n197), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(\s[20] ));
  nano23aa1n02x4               g108(.a(new_n196), .b(new_n198), .c(new_n199), .d(new_n197), .out0(new_n204));
  nona23aa1n02x4               g109(.a(new_n204), .b(new_n191), .c(new_n180), .d(new_n190), .out0(new_n205));
  nona23aa1n02x4               g110(.a(new_n199), .b(new_n197), .c(new_n196), .d(new_n198), .out0(new_n206));
  oai012aa1n02x5               g111(.a(new_n199), .b(new_n198), .c(new_n196), .o1(new_n207));
  oai012aa1n02x5               g112(.a(new_n207), .b(new_n206), .c(new_n192), .o1(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n208), .clkout(new_n209));
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n205), .c(new_n179), .d(new_n171), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  xorc02aa1n02x5               g118(.a(\a[22] ), .b(\b[21] ), .out0(new_n214));
  aoi112aa1n02x5               g119(.a(new_n212), .b(new_n214), .c(new_n210), .d(new_n213), .o1(new_n215));
  aoai13aa1n02x5               g120(.a(new_n214), .b(new_n212), .c(new_n210), .d(new_n213), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g122(.clk(\a[21] ), .clkout(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(\a[22] ), .clkout(new_n219));
  xroi22aa1d04x5               g124(.a(new_n218), .b(\b[20] ), .c(new_n219), .d(\b[21] ), .out0(new_n220));
  nanp03aa1n02x5               g125(.a(new_n220), .b(new_n188), .c(new_n204), .o1(new_n221));
  160nm_ficinv00aa1n08x5       g126(.clk(\b[21] ), .clkout(new_n222));
  oao003aa1n02x5               g127(.a(new_n219), .b(new_n222), .c(new_n212), .carry(new_n223));
  aoi012aa1n02x5               g128(.a(new_n223), .b(new_n208), .c(new_n220), .o1(new_n224));
  aoai13aa1n02x5               g129(.a(new_n224), .b(new_n221), .c(new_n179), .d(new_n171), .o1(new_n225));
  xorb03aa1n02x5               g130(.a(new_n225), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g131(.a(\b[22] ), .b(\a[23] ), .o1(new_n227));
  xorc02aa1n02x5               g132(.a(\a[23] ), .b(\b[22] ), .out0(new_n228));
  xorc02aa1n02x5               g133(.a(\a[24] ), .b(\b[23] ), .out0(new_n229));
  aoi112aa1n02x5               g134(.a(new_n227), .b(new_n229), .c(new_n225), .d(new_n228), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n229), .b(new_n227), .c(new_n225), .d(new_n228), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n231), .b(new_n230), .out0(\s[24] ));
  and002aa1n02x5               g137(.a(new_n229), .b(new_n228), .o(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n233), .clkout(new_n234));
  nano32aa1n02x4               g139(.a(new_n234), .b(new_n220), .c(new_n188), .d(new_n204), .out0(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  oaoi03aa1n02x5               g141(.a(\a[18] ), .b(\b[17] ), .c(new_n184), .o1(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n207), .clkout(new_n238));
  aoai13aa1n02x5               g143(.a(new_n220), .b(new_n238), .c(new_n204), .d(new_n237), .o1(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n223), .clkout(new_n240));
  oai022aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n241));
  aob012aa1n02x5               g146(.a(new_n241), .b(\b[23] ), .c(\a[24] ), .out0(new_n242));
  aoai13aa1n02x5               g147(.a(new_n242), .b(new_n234), .c(new_n239), .d(new_n240), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n243), .clkout(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n236), .c(new_n179), .d(new_n171), .o1(new_n245));
  xorb03aa1n02x5               g150(.a(new_n245), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g151(.a(\b[24] ), .b(\a[25] ), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[25] ), .b(\b[24] ), .out0(new_n248));
  xorc02aa1n02x5               g153(.a(\a[26] ), .b(\b[25] ), .out0(new_n249));
  aoi112aa1n02x5               g154(.a(new_n247), .b(new_n249), .c(new_n245), .d(new_n248), .o1(new_n250));
  aoai13aa1n02x5               g155(.a(new_n249), .b(new_n247), .c(new_n245), .d(new_n248), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n251), .b(new_n250), .out0(\s[26] ));
  nanp02aa1n02x5               g157(.a(new_n118), .b(new_n123), .o1(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n169), .clkout(new_n254));
  aoai13aa1n02x5               g159(.a(new_n157), .b(new_n143), .c(new_n141), .d(new_n132), .o1(new_n255));
  aoai13aa1n02x5               g160(.a(new_n178), .b(new_n254), .c(new_n255), .d(new_n158), .o1(new_n256));
  and002aa1n02x5               g161(.a(new_n249), .b(new_n248), .o(new_n257));
  nano22aa1n02x4               g162(.a(new_n221), .b(new_n233), .c(new_n257), .out0(new_n258));
  aoai13aa1n02x5               g163(.a(new_n258), .b(new_n256), .c(new_n253), .d(new_n170), .o1(new_n259));
  oai022aa1n02x5               g164(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n260));
  aob012aa1n02x5               g165(.a(new_n260), .b(\b[25] ), .c(\a[26] ), .out0(new_n261));
  aobi12aa1n02x5               g166(.a(new_n261), .b(new_n243), .c(new_n257), .out0(new_n262));
  xorc02aa1n02x5               g167(.a(\a[27] ), .b(\b[26] ), .out0(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n263), .b(new_n259), .c(new_n262), .out0(\s[27] ));
  norp02aa1n02x5               g169(.a(\b[26] ), .b(\a[27] ), .o1(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n265), .clkout(new_n266));
  aobi12aa1n02x5               g171(.a(new_n263), .b(new_n259), .c(new_n262), .out0(new_n267));
  xnrc02aa1n02x5               g172(.a(\b[27] ), .b(\a[28] ), .out0(new_n268));
  nano22aa1n02x4               g173(.a(new_n267), .b(new_n266), .c(new_n268), .out0(new_n269));
  nanp02aa1n02x5               g174(.a(new_n179), .b(new_n171), .o1(new_n270));
  aoai13aa1n02x5               g175(.a(new_n233), .b(new_n223), .c(new_n208), .d(new_n220), .o1(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n257), .clkout(new_n272));
  aoai13aa1n02x5               g177(.a(new_n261), .b(new_n272), .c(new_n271), .d(new_n242), .o1(new_n273));
  aoai13aa1n02x5               g178(.a(new_n263), .b(new_n273), .c(new_n270), .d(new_n258), .o1(new_n274));
  aoi012aa1n02x5               g179(.a(new_n268), .b(new_n274), .c(new_n266), .o1(new_n275));
  norp02aa1n02x5               g180(.a(new_n275), .b(new_n269), .o1(\s[28] ));
  norb02aa1n02x5               g181(.a(new_n263), .b(new_n268), .out0(new_n277));
  aoai13aa1n02x5               g182(.a(new_n277), .b(new_n273), .c(new_n270), .d(new_n258), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .c(new_n266), .carry(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[28] ), .b(\a[29] ), .out0(new_n280));
  aoi012aa1n02x5               g185(.a(new_n280), .b(new_n278), .c(new_n279), .o1(new_n281));
  aobi12aa1n02x5               g186(.a(new_n277), .b(new_n259), .c(new_n262), .out0(new_n282));
  nano22aa1n02x4               g187(.a(new_n282), .b(new_n279), .c(new_n280), .out0(new_n283));
  norp02aa1n02x5               g188(.a(new_n281), .b(new_n283), .o1(\s[29] ));
  xorb03aa1n02x5               g189(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g190(.a(new_n263), .b(new_n280), .c(new_n268), .out0(new_n286));
  aoai13aa1n02x5               g191(.a(new_n286), .b(new_n273), .c(new_n270), .d(new_n258), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[29] ), .b(\b[28] ), .c(new_n279), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[29] ), .b(\a[30] ), .out0(new_n289));
  aoi012aa1n02x5               g194(.a(new_n289), .b(new_n287), .c(new_n288), .o1(new_n290));
  aobi12aa1n02x5               g195(.a(new_n286), .b(new_n259), .c(new_n262), .out0(new_n291));
  nano22aa1n02x4               g196(.a(new_n291), .b(new_n288), .c(new_n289), .out0(new_n292));
  norp02aa1n02x5               g197(.a(new_n290), .b(new_n292), .o1(\s[30] ));
  norb02aa1n02x5               g198(.a(new_n286), .b(new_n289), .out0(new_n294));
  aobi12aa1n02x5               g199(.a(new_n294), .b(new_n259), .c(new_n262), .out0(new_n295));
  oao003aa1n02x5               g200(.a(\a[30] ), .b(\b[29] ), .c(new_n288), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[30] ), .b(\a[31] ), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n295), .b(new_n296), .c(new_n297), .out0(new_n298));
  aoai13aa1n02x5               g203(.a(new_n294), .b(new_n273), .c(new_n270), .d(new_n258), .o1(new_n299));
  aoi012aa1n02x5               g204(.a(new_n297), .b(new_n299), .c(new_n296), .o1(new_n300));
  norp02aa1n02x5               g205(.a(new_n300), .b(new_n298), .o1(\s[31] ));
  xnrb03aa1n02x5               g206(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g207(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n303));
  xorb03aa1n02x5               g208(.a(new_n303), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g209(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaib12aa1n02x5               g210(.a(new_n120), .b(new_n116), .c(new_n109), .out0(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_ficinv00aa1n08x5       g212(.clk(new_n112), .clkout(new_n308));
  160nm_ficinv00aa1n08x5       g213(.clk(\a[6] ), .clkout(new_n309));
  160nm_ficinv00aa1n08x5       g214(.clk(\b[5] ), .clkout(new_n310));
  oaoi03aa1n02x5               g215(.a(new_n309), .b(new_n310), .c(new_n306), .o1(new_n311));
  xnbna2aa1n03x5               g216(.a(new_n311), .b(new_n308), .c(new_n113), .out0(\s[7] ));
  oaoi03aa1n02x5               g217(.a(\a[7] ), .b(\b[6] ), .c(new_n311), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g219(.a(new_n124), .b(new_n118), .c(new_n123), .out0(\s[9] ));
endmodule


