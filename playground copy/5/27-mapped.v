// Benchmark "adder" written by ABC on Thu Jul 11 11:16:12 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n248, new_n249, new_n250, new_n251, new_n252, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n302, new_n305,
    new_n306, new_n308, new_n309, new_n311;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  and002aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(new_n99), .clkout(new_n100));
  and002aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\a[3] ), .clkout(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(\b[2] ), .clkout(new_n103));
  nanp02aa1n02x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(new_n104), .b(new_n105), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  oai012aa1n02x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  oa0022aa1n02x5               g015(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n111));
  oaoi13aa1n02x5               g016(.a(new_n101), .b(new_n111), .c(new_n110), .d(new_n106), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nona23aa1n02x4               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .out0(new_n118));
  xnrc02aa1n02x5               g023(.a(\b[4] ), .b(\a[5] ), .out0(new_n119));
  norp03aa1n02x5               g024(.a(new_n117), .b(new_n118), .c(new_n119), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n112), .b(new_n120), .o1(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(new_n113), .clkout(new_n122));
  nanp02aa1n02x5               g027(.a(new_n115), .b(new_n114), .o1(new_n123));
  aoi112aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n124));
  oab012aa1n02x4               g029(.a(new_n124), .b(\a[6] ), .c(\b[5] ), .out0(new_n125));
  oai112aa1n02x5               g030(.a(new_n122), .b(new_n123), .c(new_n117), .d(new_n125), .o1(new_n126));
  xnrc02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .out0(new_n127));
  nona22aa1n02x4               g032(.a(new_n121), .b(new_n126), .c(new_n127), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n98), .b(new_n128), .c(new_n100), .out0(\s[10] ));
  nanp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  aob012aa1n02x5               g035(.a(new_n97), .b(new_n128), .c(new_n100), .out0(new_n131));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xobna2aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n130), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n132), .clkout(new_n136));
  nano22aa1n02x4               g041(.a(new_n132), .b(new_n130), .c(new_n133), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n137), .b(new_n98), .c(new_n128), .d(new_n100), .o1(new_n138));
  xorc02aa1n02x5               g043(.a(\a[12] ), .b(\b[11] ), .out0(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n139), .b(new_n138), .c(new_n136), .out0(\s[12] ));
  oai022aa1n02x5               g045(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n141));
  nona23aa1n02x4               g046(.a(new_n137), .b(new_n139), .c(new_n141), .d(new_n99), .out0(new_n142));
  160nm_ficinv00aa1n08x5       g047(.clk(new_n142), .clkout(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n144));
  norp02aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  aoi112aa1n02x5               g050(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n146));
  aoi113aa1n02x5               g051(.a(new_n146), .b(new_n145), .c(new_n137), .d(new_n139), .e(new_n141), .o1(new_n147));
  xnrc02aa1n02x5               g052(.a(\b[12] ), .b(\a[13] ), .out0(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n148), .clkout(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n144), .c(new_n147), .out0(\s[13] ));
  orn002aa1n02x5               g055(.a(\a[13] ), .b(\b[12] ), .o(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n148), .c(new_n144), .d(new_n147), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g058(.a(\b[13] ), .b(\a[14] ), .out0(new_n154));
  norp02aa1n02x5               g059(.a(new_n154), .b(new_n148), .o1(new_n155));
  160nm_ficinv00aa1n08x5       g060(.clk(new_n155), .clkout(new_n156));
  oaoi03aa1n02x5               g061(.a(\a[14] ), .b(\b[13] ), .c(new_n151), .o1(new_n157));
  160nm_ficinv00aa1n08x5       g062(.clk(new_n157), .clkout(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n156), .c(new_n144), .d(new_n147), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  norp02aa1n02x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  aoi112aa1n02x5               g070(.a(new_n165), .b(new_n161), .c(new_n159), .d(new_n162), .o1(new_n166));
  aoai13aa1n02x5               g071(.a(new_n165), .b(new_n161), .c(new_n159), .d(new_n162), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(\s[16] ));
  nano23aa1n02x4               g073(.a(new_n161), .b(new_n163), .c(new_n164), .d(new_n162), .out0(new_n169));
  nona22aa1n02x4               g074(.a(new_n169), .b(new_n154), .c(new_n148), .out0(new_n170));
  norp02aa1n02x5               g075(.a(new_n170), .b(new_n142), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n171), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n172));
  nanp03aa1n02x5               g077(.a(new_n137), .b(new_n139), .c(new_n141), .o1(new_n173));
  nona22aa1n02x4               g078(.a(new_n173), .b(new_n146), .c(new_n145), .out0(new_n174));
  norb03aa1n02x5               g079(.a(new_n169), .b(new_n148), .c(new_n154), .out0(new_n175));
  nanp02aa1n02x5               g080(.a(new_n161), .b(new_n164), .o1(new_n176));
  nanp02aa1n02x5               g081(.a(new_n169), .b(new_n157), .o1(new_n177));
  oai112aa1n02x5               g082(.a(new_n177), .b(new_n176), .c(\b[15] ), .d(\a[16] ), .o1(new_n178));
  aoi012aa1n02x5               g083(.a(new_n178), .b(new_n174), .c(new_n175), .o1(new_n179));
  xnrc02aa1n02x5               g084(.a(\b[16] ), .b(\a[17] ), .out0(new_n180));
  xobna2aa1n03x5               g085(.a(new_n180), .b(new_n172), .c(new_n179), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g086(.clk(\a[17] ), .clkout(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(\b[16] ), .clkout(new_n183));
  nanp02aa1n02x5               g088(.a(new_n183), .b(new_n182), .o1(new_n184));
  aoai13aa1n02x5               g089(.a(new_n184), .b(new_n180), .c(new_n172), .d(new_n179), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[18] ), .clkout(new_n187));
  xroi22aa1d04x5               g092(.a(new_n182), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(new_n188), .clkout(new_n189));
  norp02aa1n02x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nanp02aa1n02x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  aoi013aa1n02x4               g096(.a(new_n190), .b(new_n191), .c(new_n182), .d(new_n183), .o1(new_n192));
  aoai13aa1n02x5               g097(.a(new_n192), .b(new_n189), .c(new_n172), .d(new_n179), .o1(new_n193));
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
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n205), .c(new_n172), .d(new_n179), .o1(new_n210));
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
  oaoi03aa1n02x5               g126(.a(\a[18] ), .b(\b[17] ), .c(new_n184), .o1(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n207), .clkout(new_n223));
  aoai13aa1n02x5               g128(.a(new_n220), .b(new_n223), .c(new_n204), .d(new_n222), .o1(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(\b[21] ), .clkout(new_n225));
  oao003aa1n02x5               g130(.a(new_n219), .b(new_n225), .c(new_n212), .carry(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n226), .clkout(new_n227));
  nanp02aa1n02x5               g132(.a(new_n224), .b(new_n227), .o1(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(new_n228), .clkout(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n221), .c(new_n172), .d(new_n179), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xorc02aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  aoi112aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(\s[24] ));
  and002aa1n02x5               g142(.a(new_n234), .b(new_n233), .o(new_n238));
  nanb03aa1n02x5               g143(.a(new_n205), .b(new_n238), .c(new_n220), .out0(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n238), .clkout(new_n240));
  nanp02aa1n02x5               g145(.a(\b[23] ), .b(\a[24] ), .o1(new_n241));
  oai022aa1n02x5               g146(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n242));
  nanp02aa1n02x5               g147(.a(new_n242), .b(new_n241), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n240), .c(new_n224), .d(new_n227), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  aoai13aa1n02x5               g150(.a(new_n245), .b(new_n239), .c(new_n172), .d(new_n179), .o1(new_n246));
  xorb03aa1n02x5               g151(.a(new_n246), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g152(.a(\b[24] ), .b(\a[25] ), .o1(new_n248));
  xorc02aa1n02x5               g153(.a(\a[25] ), .b(\b[24] ), .out0(new_n249));
  xorc02aa1n02x5               g154(.a(\a[26] ), .b(\b[25] ), .out0(new_n250));
  aoi112aa1n02x5               g155(.a(new_n248), .b(new_n250), .c(new_n246), .d(new_n249), .o1(new_n251));
  aoai13aa1n02x5               g156(.a(new_n250), .b(new_n248), .c(new_n246), .d(new_n249), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n252), .b(new_n251), .out0(\s[26] ));
  nanb02aa1n02x5               g158(.a(new_n126), .b(new_n121), .out0(new_n254));
  oabi12aa1n02x5               g159(.a(new_n178), .b(new_n147), .c(new_n170), .out0(new_n255));
  and002aa1n02x5               g160(.a(new_n250), .b(new_n249), .o(new_n256));
  nano22aa1n02x4               g161(.a(new_n221), .b(new_n238), .c(new_n256), .out0(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n255), .c(new_n254), .d(new_n171), .o1(new_n258));
  oai022aa1n02x5               g163(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n259));
  aob012aa1n02x5               g164(.a(new_n259), .b(\b[25] ), .c(\a[26] ), .out0(new_n260));
  aobi12aa1n02x5               g165(.a(new_n260), .b(new_n244), .c(new_n256), .out0(new_n261));
  xorc02aa1n02x5               g166(.a(\a[27] ), .b(\b[26] ), .out0(new_n262));
  xnbna2aa1n03x5               g167(.a(new_n262), .b(new_n261), .c(new_n258), .out0(\s[27] ));
  norp02aa1n02x5               g168(.a(\b[26] ), .b(\a[27] ), .o1(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n264), .clkout(new_n265));
  aobi12aa1n02x5               g170(.a(new_n262), .b(new_n261), .c(new_n258), .out0(new_n266));
  xnrc02aa1n02x5               g171(.a(\b[27] ), .b(\a[28] ), .out0(new_n267));
  nano22aa1n02x4               g172(.a(new_n266), .b(new_n265), .c(new_n267), .out0(new_n268));
  aobi12aa1n02x5               g173(.a(new_n257), .b(new_n172), .c(new_n179), .out0(new_n269));
  aoai13aa1n02x5               g174(.a(new_n238), .b(new_n226), .c(new_n208), .d(new_n220), .o1(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n256), .clkout(new_n271));
  aoai13aa1n02x5               g176(.a(new_n260), .b(new_n271), .c(new_n270), .d(new_n243), .o1(new_n272));
  oai012aa1n02x5               g177(.a(new_n262), .b(new_n272), .c(new_n269), .o1(new_n273));
  aoi012aa1n02x5               g178(.a(new_n267), .b(new_n273), .c(new_n265), .o1(new_n274));
  norp02aa1n02x5               g179(.a(new_n274), .b(new_n268), .o1(\s[28] ));
  norb02aa1n02x5               g180(.a(new_n262), .b(new_n267), .out0(new_n276));
  oai012aa1n02x5               g181(.a(new_n276), .b(new_n272), .c(new_n269), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[28] ), .b(\b[27] ), .c(new_n265), .carry(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[28] ), .b(\a[29] ), .out0(new_n279));
  aoi012aa1n02x5               g184(.a(new_n279), .b(new_n277), .c(new_n278), .o1(new_n280));
  aobi12aa1n02x5               g185(.a(new_n276), .b(new_n261), .c(new_n258), .out0(new_n281));
  nano22aa1n02x4               g186(.a(new_n281), .b(new_n278), .c(new_n279), .out0(new_n282));
  norp02aa1n02x5               g187(.a(new_n280), .b(new_n282), .o1(\s[29] ));
  xorb03aa1n02x5               g188(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g189(.a(new_n262), .b(new_n279), .c(new_n267), .out0(new_n285));
  oai012aa1n02x5               g190(.a(new_n285), .b(new_n272), .c(new_n269), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[29] ), .b(\b[28] ), .c(new_n278), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[29] ), .b(\a[30] ), .out0(new_n288));
  aoi012aa1n02x5               g193(.a(new_n288), .b(new_n286), .c(new_n287), .o1(new_n289));
  aobi12aa1n02x5               g194(.a(new_n285), .b(new_n261), .c(new_n258), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n287), .c(new_n288), .out0(new_n291));
  norp02aa1n02x5               g196(.a(new_n289), .b(new_n291), .o1(\s[30] ));
  norb02aa1n02x5               g197(.a(new_n285), .b(new_n288), .out0(new_n293));
  aobi12aa1n02x5               g198(.a(new_n293), .b(new_n261), .c(new_n258), .out0(new_n294));
  oao003aa1n02x5               g199(.a(\a[30] ), .b(\b[29] ), .c(new_n287), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[30] ), .b(\a[31] ), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n294), .b(new_n295), .c(new_n296), .out0(new_n297));
  oai012aa1n02x5               g202(.a(new_n293), .b(new_n272), .c(new_n269), .o1(new_n298));
  aoi012aa1n02x5               g203(.a(new_n296), .b(new_n298), .c(new_n295), .o1(new_n299));
  norp02aa1n02x5               g204(.a(new_n299), .b(new_n297), .o1(\s[31] ));
  xnbna2aa1n03x5               g205(.a(new_n110), .b(new_n104), .c(new_n105), .out0(\s[3] ));
  oaoi03aa1n02x5               g206(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n302));
  xorb03aa1n02x5               g207(.a(new_n302), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g208(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g209(.a(\a[5] ), .b(\b[4] ), .o(new_n305));
  nanb02aa1n02x5               g210(.a(new_n119), .b(new_n112), .out0(new_n306));
  xobna2aa1n03x5               g211(.a(new_n118), .b(new_n306), .c(new_n305), .out0(\s[6] ));
  norp02aa1n02x5               g212(.a(new_n119), .b(new_n118), .o1(new_n308));
  aobi12aa1n02x5               g213(.a(new_n125), .b(new_n112), .c(new_n308), .out0(new_n309));
  xnrb03aa1n02x5               g214(.a(new_n309), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g215(.a(\a[7] ), .b(\b[6] ), .c(new_n309), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g217(.a(new_n254), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


